#!/usr/bin/env python3
"""
STM32L051 Modbus RTU 配置上位机
基于项目寄存器地址表，提供 Web UI 快速配置
用法: python app.py [端口]  (默认 8080)
依赖: pip install pyserial
"""
import sys
import json
import struct
import threading
import time
import socketserver
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs

# ═══════════════════════════════════════════════════════════════
#  Modbus RTU 协议层
# ═══════════════════════════════════════════════════════════════

CRC16_TABLE = []
def _build_crc_table():
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
        CRC16_TABLE.append(crc)
_build_crc_table()

def modbus_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc = (crc >> 8) ^ CRC16_TABLE[(crc ^ b) & 0xFF]
    return crc

class ModbusRTU:
    def __init__(self):
        self.serial = None
        self.lock = threading.Lock()
        self.connected = False
        self.port_name = ""
        self.baudrate = 9600
        self.timeout = 1.0

    def connect(self, port, baudrate=9600, parity='N', stopbits=1, timeout=1.0):
        import serial
        self.disconnect()
        parity_map = {'N': serial.PARITY_NONE, 'O': serial.PARITY_ODD, 'E': serial.PARITY_EVEN}
        self.serial = serial.Serial(
            port=port, baudrate=baudrate,
            bytesize=8, parity=parity_map.get(parity, serial.PARITY_NONE),
            stopbits=stopbits, timeout=timeout
        )
        self.connected = True
        self.port_name = port
        self.baudrate = baudrate
        self.timeout = timeout
        return True

    def disconnect(self):
        if self.serial and self.serial.is_open:
            self.serial.close()
        self.connected = False
        self.serial = None

    def read_holding_regs(self, slave_addr, start_addr, count, fc=0x03):
        """读保持/输入寄存器，返回 [uint16, ...] 或抛异常"""
        with self.lock:
            if not self.connected:
                raise ConnectionError("未连接")
            frame = struct.pack('>BBHH', slave_addr, fc, start_addr, count)
            frame += struct.pack('<H', modbus_crc16(frame))
            self.serial.reset_input_buffer()
            self.serial.write(frame)
            time.sleep(0.05)
            resp = self.serial.read(3 + count * 2 + 2)
            if len(resp) < 5:
                raise TimeoutError("响应超时或数据不足")
            if resp[1] & 0x80:
                raise Exception(f"Modbus 异常: 功能码=0x{resp[1]:02X} 异常码=0x{resp[2]:02X}")
            byte_count = resp[2]
            if byte_count != count * 2:
                raise ValueError(f"字节数不匹配: 期望{count*2}, 收到{byte_count}")
            rx_crc = resp[-2] | (resp[-1] << 8)
            if rx_crc != modbus_crc16(resp[:-2]):
                raise ValueError("CRC 校验失败")
            regs = []
            for i in range(count):
                regs.append((resp[3 + i*2] << 8) | resp[4 + i*2])
            return regs

    def write_single_reg(self, slave_addr, reg_addr, value):
        """0x06 写单个寄存器"""
        with self.lock:
            if not self.connected:
                raise ConnectionError("未连接")
            frame = struct.pack('>BBHH', slave_addr, 0x06, reg_addr, value)
            frame += struct.pack('<H', modbus_crc16(frame))
            self.serial.reset_input_buffer()
            self.serial.write(frame)
            time.sleep(0.05)
            resp = self.serial.read(8)
            if len(resp) < 8:
                raise TimeoutError("响应超时")
            if resp[1] & 0x80:
                raise Exception(f"Modbus 异常: 异常码=0x{resp[2]:02X}")
            rx_crc = resp[-2] | (resp[-1] << 8)
            if rx_crc != modbus_crc16(resp[:-2]):
                raise ValueError("CRC 校验失败")
            return True

    def write_multiple_regs(self, slave_addr, start_addr, values):
        """0x10 写多个寄存器"""
        with self.lock:
            if not self.connected:
                raise ConnectionError("未连接")
            count = len(values)
            byte_count = count * 2
            data = b''
            for v in values:
                data += struct.pack('>H', v)
            frame = struct.pack('>BBHHB', slave_addr, 0x10, start_addr, count, byte_count)
            frame += data
            frame += struct.pack('<H', modbus_crc16(frame))
            self.serial.reset_input_buffer()
            self.serial.write(frame)
            time.sleep(0.05)
            resp = self.serial.read(8)
            if len(resp) < 8:
                raise TimeoutError("响应超时")
            if resp[1] & 0x80:
                raise Exception(f"Modbus 异常: 异常码=0x{resp[2]:02X}")
            rx_crc = resp[-2] | (resp[-1] << 8)
            if rx_crc != modbus_crc16(resp[:-2]):
                raise ValueError("CRC 校验失败")
            return True


# ═══════════════════════════════════════════════════════════════
#  寄存器地址映射 (与固件一致)
# ═══════════════════════════════════════════════════════════════

SYS_REGS = {
    'uart2_baudrate_lo': 0x0000,
    'uart2_baudrate_hi': 0x0001,
    'uart2_parity':      0x0002,
    'uart2_stopbits':    0x0003,
    'rs485_de_delay':    0x0004,
    'rs485_re_delay':    0x0005,
    'local_mb_addr':     0x0006,
    'slave_count':       0x0007,
    'report_format':     0x0008,
    'uart1_baudrate_lo': 0x0009,
    'uart1_baudrate_hi': 0x000A,
}

def slave_base(n): return 0x0100 + n * 0x0200
def slave_dp_base(n, m): return slave_base(n) + 0x0010 + m * 0x0010
def slave_name_base(n): return slave_base(n) + 0x0005
def slave_dp_name_base(n, m): return slave_dp_base(n, m) + 0x0003


# ═══════════════════════════════════════════════════════════════
#  全局实例
# ═══════════════════════════════════════════════════════════════

mb = ModbusRTU()


# ═══════════════════════════════════════════════════════════════
#  Web 服务器
# ═══════════════════════════════════════════════════════════════

HTML_PAGE = r'''<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>STM32L051 Modbus 配置工具</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
:root{
  --bg:#0f1117;--card:#1a1d27;--border:#2a2d3a;--text:#e0e0e0;
  --dim:#888;--accent:#4fc3f7;--accent2:#81c784;--warn:#ffb74d;
  --err:#ef5350;--input-bg:#12141c;--hover:#252838;
}
body{font-family:'Segoe UI','PingFang SC',sans-serif;background:var(--bg);color:var(--text);min-height:100vh}
.header{background:linear-gradient(135deg,#1a237e,#0d47a1);padding:20px 32px;display:flex;align-items:center;gap:16px;box-shadow:0 2px 12px rgba(0,0,0,.4)}
.header h1{font-size:20px;font-weight:600;letter-spacing:.5px}
.header .badge{background:rgba(255,255,255,.15);padding:4px 12px;border-radius:12px;font-size:12px}
.container{max-width:1100px;margin:0 auto;padding:20px}

/* 连接面板 */
.conn-panel{background:var(--card);border:1px solid var(--border);border-radius:12px;padding:20px;margin-bottom:20px;display:flex;flex-wrap:wrap;gap:12px;align-items:end}
.conn-panel label{font-size:13px;color:var(--dim);display:block;margin-bottom:4px}
.conn-panel select,.conn-panel input{background:var(--input-bg);border:1px solid var(--border);color:var(--text);padding:8px 12px;border-radius:6px;font-size:14px}
.conn-panel select{min-width:120px}
.conn-panel input{width:100px}
.btn{padding:8px 20px;border:none;border-radius:6px;font-size:14px;cursor:pointer;font-weight:500;transition:all .15s}
.btn-primary{background:var(--accent);color:#000}.btn-primary:hover{background:#29b6f6}
.btn-success{background:var(--accent2);color:#000}.btn-success:hover{background:#66bb6a}
.btn-warn{background:var(--warn);color:#000}.btn-warn:hover{background:#ffa726}
.btn-danger{background:var(--err);color:#fff}.btn-danger:hover{background:#e53935}
.btn:disabled{opacity:.4;cursor:not-allowed}
.status-dot{display:inline-block;width:10px;height:10px;border-radius:50%;margin-right:6px}
.status-dot.on{background:var(--accent2);box-shadow:0 0 6px var(--accent2)}
.status-dot.off{background:var(--err)}

/* 区块 */
.section{background:var(--card);border:1px solid var(--border);border-radius:12px;margin-bottom:16px;overflow:hidden}
.section-header{padding:14px 20px;background:rgba(255,255,255,.03);display:flex;justify-content:space-between;align-items:center;cursor:pointer;user-select:none}
.section-header:hover{background:rgba(255,255,255,.06)}
.section-header h3{font-size:15px;font-weight:600}
.section-body{padding:16px 20px;display:grid;grid-template-columns:repeat(auto-fill,minmax(240px,1fr));gap:12px}
.section-body.collapsed{display:none}

/* 表单项 */
.field{display:flex;flex-direction:column;gap:4px}
.field label{font-size:12px;color:var(--dim);font-weight:500}
.field input,.field select{background:var(--input-bg);border:1px solid var(--border);color:var(--text);padding:8px 10px;border-radius:6px;font-size:13px;transition:border-color .2s}
.field input:focus,.field select:focus{outline:none;border-color:var(--accent)}
.field .hint{font-size:11px;color:#666}

/* Tab */
.tabs{display:flex;gap:0;border-bottom:2px solid var(--border);padding:0 20px}
.tab{padding:10px 20px;cursor:pointer;font-size:13px;color:var(--dim);border-bottom:2px solid transparent;margin-bottom:-2px;transition:all .2s}
.tab:hover{color:var(--text)}
.tab.active{color:var(--accent);border-bottom-color:var(--accent)}
.tab-content{display:none}.tab-content.active{display:block}

/* 操作栏 */
.actions{display:flex;gap:10px;margin:16px 0;flex-wrap:wrap}
.toast{position:fixed;top:20px;right:20px;padding:12px 20px;border-radius:8px;font-size:13px;z-index:999;opacity:0;transition:opacity .3s;pointer-events:none;max-width:400px}
.toast.show{opacity:1}
.toast.success{background:#1b5e20;color:#a5d6a7;border:1px solid #2e7d32}
.toast.error{background:#b71c1c;color:#ef9a9a;border:1px solid #c62828}
.toast.info{background:#0d47a1;color:#90caf9;border:1px solid #1565c0}

/* 日志 */
.log-panel{background:var(--card);border:1px solid var(--border);border-radius:12px;margin-top:16px}
.log-panel .section-header{border-bottom:1px solid var(--border)}
.log-body{padding:12px 20px;max-height:200px;overflow-y:auto;font-family:'Cascadia Code','Fira Code',monospace;font-size:12px;line-height:1.8;color:#aaa}
.log-body .ok{color:var(--accent2)}.log-body .err{color:var(--err)}.log-body .wrn{color:var(--warn)}

/* JSON */
.json-area{width:100%;min-height:120px;background:var(--input-bg);border:1px solid var(--border);color:var(--text);font-family:monospace;font-size:12px;padding:10px;border-radius:6px;resize:vertical}

/* 响应式 */
@media(max-width:600px){
  .conn-panel{flex-direction:column}
  .section-body{grid-template-columns:1fr}
  .header{padding:16px}
  .container{padding:12px}
}
</style>
</head>
<body>

<div class="header">
  <h1>⚡ STM32L051 Modbus 配置工具</h1>
  <span class="badge">v2.2</span>
</div>

<div class="container">
  <!-- 连接 -->
  <div class="conn-panel">
    <div>
      <label>串口</label>
      <select id="port"><option value="">扫描中...</option></select>
    </div>
    <div>
      <label>波特率</label>
      <select id="conn-baud">
        <option value="9600" selected>9600</option>
        <option value="19200">19200</option>
        <option value="38400">38400</option>
        <option value="57600">57600</option>
        <option value="115200">115200</option>
      </select>
    </div>
    <div>
      <label>校验</label>
      <select id="conn-parity">
        <option value="N" selected>无</option>
        <option value="O">奇</option>
        <option value="E">偶</option>
      </select>
    </div>
    <div>
      <label>从站地址</label>
      <input type="number" id="mb-addr" value="1" min="1" max="247">
    </div>
    <div style="display:flex;gap:8px;align-items:end">
      <button class="btn btn-primary" id="btn-connect" onclick="doConnect()">连接</button>
      <button class="btn btn-danger" id="btn-disconnect" onclick="doDisconnect()" disabled>断开</button>
    </div>
    <div style="display:flex;align-items:end;gap:6px">
      <span class="status-dot off" id="status-dot"></span>
      <span id="status-text" style="font-size:13px;color:var(--dim)">未连接</span>
    </div>
  </div>

  <!-- 操作 -->
  <div class="actions">
    <button class="btn btn-success" onclick="readAll()" id="btn-read" disabled>📥 读取全部配置</button>
    <button class="btn btn-warn" onclick="writeAll()" id="btn-write" disabled>📤 写入全部配置</button>
    <button class="btn btn-primary" onclick="exportJSON()">💾 导出 JSON</button>
    <button class="btn btn-primary" onclick="importJSON()">📂 导入 JSON</button>
  </div>

  <!-- 系统配置 -->
  <div class="section">
    <div class="section-header" onclick="toggleSection(this)">
      <h3>⚙️ 系统配置</h3>
      <span style="color:var(--dim)">▼</span>
    </div>
    <div class="section-body" id="sys-fields">
      <div class="field">
        <label>UART2 波特率 (Modbus)</label>
        <input type="number" id="uart2_baudrate" value="9600" step="300" min="300" max="115200">
      </div>
      <div class="field">
        <label>UART2 校验位</label>
        <select id="uart2_parity">
          <option value="0">无</option><option value="1">奇</option><option value="2">偶</option>
        </select>
      </div>
      <div class="field">
        <label>UART2 停止位</label>
        <select id="uart2_stopbits"><option value="1">1</option><option value="2">2</option></select>
      </div>
      <div class="field">
        <label>RS485 DE 延时 (μs)</label>
        <input type="number" id="rs485_de_delay" value="50" min="0" max="65535">
      </div>
      <div class="field">
        <label>RS485 RE 延时 (μs)</label>
        <input type="number" id="rs485_re_delay" value="50" min="0" max="65535">
      </div>
      <div class="field">
        <label>本机 Modbus 地址</label>
        <input type="number" id="local_mb_addr" value="1" min="1" max="247">
      </div>
      <div class="field">
        <label>采集从机数</label>
        <input type="number" id="slave_count" value="1" min="1" max="5">
      </div>
      <div class="field">
        <label>上报格式</label>
        <select id="report_format">
          <option value="0">TEXT</option><option value="1">JSON</option><option value="2">HEX</option>
        </select>
      </div>
      <div class="field">
        <label>UART1 波特率 (上报)</label>
        <input type="number" id="uart1_baudrate" value="9600" step="300" min="300" max="115200">
      </div>
    </div>
  </div>

  <!-- 从机配置 -->
  <div class="section">
    <div class="section-header" onclick="toggleSection(this)">
      <h3>📡 从机配置</h3>
      <span style="color:var(--dim)">▼</span>
    </div>
    <div class="tabs" id="slave-tabs"></div>
    <div id="slave-panels"></div>
  </div>

  <!-- JSON -->
  <div class="section">
    <div class="section-header" onclick="toggleSection(this)">
      <h3>📋 JSON 配置</h3>
      <span style="color:var(--dim)">▼</span>
    </div>
    <div style="padding:16px 20px">
      <textarea class="json-area" id="json-area" placeholder="点击「导出 JSON」查看当前配置，或粘贴配置后点击「导入 JSON」"></textarea>
    </div>
  </div>

  <!-- 日志 -->
  <div class="log-panel">
    <div class="section-header" onclick="toggleSection(this)">
      <h3>📜 通信日志</h3>
      <span style="color:var(--dim)">▼</span>
    </div>
    <div class="log-body" id="log-body"></div>
  </div>
</div>

<div class="toast" id="toast"></div>

<!-- 导入 JSON 的隐藏文件输入 -->
<input type="file" id="file-import" accept=".json" style="display:none" onchange="handleImport(event)">

<script>
// ═══════════════════════════════════════
//  状态
// ═══════════════════════════════════════
let connected = false;
const SLAVE_COUNT = 5;
const DP_COUNT = 8;

// ═══════════════════════════════════════
//  初始化 UI
// ═══════════════════════════════════════
function initSlaveTabs() {
  const tabsEl = document.getElementById('slave-tabs');
  const panelsEl = document.getElementById('slave-panels');
  let tabsHTML = '', panelsHTML = '';
  for (let s = 0; s < SLAVE_COUNT; s++) {
    tabsHTML += `<div class="tab${s===0?' active':''}" onclick="switchTab(${s})">从机 ${s+1}</div>`;
    panelsHTML += `<div class="tab-content${s===0?' active':''}" id="slave-panel-${s}">`;
    panelsHTML += `<div class="section-body">`;
    // 基本配置
    panelsHTML += `
      <div class="field"><label>Modbus 地址</label><input type="number" id="s${s}_addr" value="1" min="1" max="247"></div>
      <div class="field"><label>启用</label><select id="s${s}_enabled"><option value="1">启用</option><option value="0" selected>停用</option></select></div>
      <div class="field"><label>数据点数量</label><input type="number" id="s${s}_dp_count" value="1" min="0" max="8"></div>
      <div class="field"><label>轮询周期 (ms)</label><input type="number" id="s${s}_poll" value="1000" min="100" max="4294967295"></div>
      <div class="field"><label>设备名称</label><input type="text" id="s${s}_name" maxlength="20" placeholder="如: 1号温湿度"></div>
    `;
    panelsHTML += `</div>`;
    // 数据点
    for (let p = 0; p < DP_COUNT; p++) {
      panelsHTML += `<div style="padding:0 20px 12px"><div style="display:grid;grid-template-columns:repeat(auto-fill,minmax(180px,1fr));gap:10px;padding:10px 0;border-top:1px solid var(--border)">`;
      panelsHTML += `<div style="grid-column:1/-1;font-size:12px;color:var(--accent);font-weight:600">数据点 P${p+1}</div>`;
      panelsHTML += `
        <div class="field"><label>寄存器地址</label><input type="number" id="s${s}_p${p}_reg" value="0" min="0" max="65535"></div>
        <div class="field"><label>数据类型</label><select id="s${s}_p${p}_type">
          <option value="0">U16</option><option value="1">I16</option><option value="2">U32</option><option value="3">I32</option><option value="4">Float</option></select></div>
        <div class="field"><label>字节序</label><select id="s${s}_p${p}_order">
          <option value="0">ABCD</option><option value="1">BADC</option><option value="2">CDAB</option><option value="3">DCBA</option></select></div>
        <div class="field"><label>名称</label><input type="text" id="s${s}_p${p}_name" maxlength="20" placeholder="如: 温度"></div>
      `;
      panelsHTML += `</div></div>`;
    }
    panelsHTML += `</div>`;
  }
  tabsEl.innerHTML = tabsHTML;
  panelsEl.innerHTML = panelsHTML;
}

function switchTab(s) {
  document.querySelectorAll('.tab').forEach((t,i) => t.classList.toggle('active', i===s));
  document.querySelectorAll('.tab-content').forEach((p,i) => p.classList.toggle('active', i===s));
}

function toggleSection(el) {
  const body = el.nextElementSibling;
  if (body) body.classList.toggle('collapsed');
  const arrow = el.querySelector('span');
  if (arrow) arrow.textContent = body.classList.contains('collapsed') ? '▶' : '▼';
}

// ═══════════════════════════════════════
//  日志
// ═══════════════════════════════════════
function log(msg, cls='') {
  const el = document.getElementById('log-body');
  const t = new Date().toLocaleTimeString();
  el.innerHTML += `<div class="${cls}">[${t}] ${msg}</div>`;
  el.scrollTop = el.scrollHeight;
}

function toast(msg, type='info') {
  const el = document.getElementById('toast');
  el.textContent = msg;
  el.className = `toast ${type} show`;
  setTimeout(() => el.classList.remove('show'), 3000);
}

// ═══════════════════════════════════════
//  API 调用
// ═══════════════════════════════════════
async function api(path, body=null) {
  const opts = body ? {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(body)} : {};
  const resp = await fetch(path, opts);
  const data = await resp.json();
  if (!resp.ok || data.error) throw new Error(data.error || `HTTP ${resp.status}`);
  return data;
}

async function scanPorts() {
  try {
    const data = await api('/api/ports');
    const sel = document.getElementById('port');
    sel.innerHTML = '';
    if (data.ports.length === 0) {
      sel.innerHTML = '<option value="">无可用串口</option>';
    } else {
      data.ports.forEach(p => {
        sel.innerHTML += `<option value="${p}">${p}</option>`;
      });
    }
  } catch(e) {
    document.getElementById('port').innerHTML = '<option value="">扫描失败</option>';
  }
}

async function doConnect() {
  const port = document.getElementById('port').value;
  if (!port) { toast('请选择串口', 'error'); return; }
  try {
    const data = await api('/api/connect', {
      port: port,
      baudrate: parseInt(document.getElementById('conn-baud').value),
      parity: document.getElementById('conn-parity').value
    });
    connected = true;
    document.getElementById('status-dot').className = 'status-dot on';
    document.getElementById('status-text').textContent = `已连接 ${port}`;
    document.getElementById('btn-connect').disabled = true;
    document.getElementById('btn-disconnect').disabled = false;
    document.getElementById('btn-read').disabled = false;
    document.getElementById('btn-write').disabled = false;
    log(`已连接 ${port} @ ${document.getElementById('conn-baud').value}`, 'ok');
    toast('连接成功', 'success');
  } catch(e) {
    log(`连接失败: ${e.message}`, 'err');
    toast('连接失败: ' + e.message, 'error');
  }
}

async function doDisconnect() {
  await api('/api/disconnect');
  connected = false;
  document.getElementById('status-dot').className = 'status-dot off';
  document.getElementById('status-text').textContent = '未连接';
  document.getElementById('btn-connect').disabled = false;
  document.getElementById('btn-disconnect').disabled = true;
  document.getElementById('btn-read').disabled = true;
  document.getElementById('btn-write').disabled = true;
  log('已断开连接', 'wrn');
}

// ═══════════════════════════════════════
//  读取全部配置
// ═══════════════════════════════════════
async function readAll() {
  const addr = parseInt(document.getElementById('mb-addr').value);
  try {
    log('开始读取全部配置...', 'ok');
    // 系统寄存器: 0x0000 ~ 0x000A (11个)
    const sysRegs = await api('/api/read', {addr, start: 0x0000, count: 11});
    const r = sysRegs.regs;
    document.getElementById('uart2_baudrate').value = r[0] | (r[1] << 16);
    document.getElementById('uart2_parity').value = r[2];
    document.getElementById('uart2_stopbits').value = r[3];
    document.getElementById('rs485_de_delay').value = r[4];
    document.getElementById('rs485_re_delay').value = r[5];
    document.getElementById('local_mb_addr').value = r[6];
    document.getElementById('slave_count').value = r[7];
    document.getElementById('report_format').value = r[8];
    document.getElementById('uart1_baudrate').value = r[9] | (r[10] << 16);
    log('系统寄存器读取完成 (0x0000~0x000A)', 'ok');

    // 从机配置
    for (let s = 0; s < SLAVE_COUNT; s++) {
      const base = 0x0100 + s * 0x0200;
      // 读基本配置 (5) + 名称 (10) = 15
      const slaveRegs = await api('/api/read', {addr, start: base, count: 15});
      const sr = slaveRegs.regs;
      document.getElementById(`s${s}_addr`).value = sr[0];
      document.getElementById(`s${s}_enabled`).value = sr[1];
      document.getElementById(`s${s}_dp_count`).value = sr[2];
      document.getElementById(`s${s}_poll`).value = sr[3] | (sr[4] << 16);
      // 名称: sr[5]~sr[14] → 20字节
      document.getElementById(`s${s}_name`).value = regsToString(sr.slice(5, 15));

      // 数据点
      for (let p = 0; p < DP_COUNT; p++) {
        const dpBase = base + 0x0010 + p * 0x0010;
        const dpRegs = await api('/api/read', {addr, start: dpBase, count: 13});
        const dr = dpRegs.regs;
        document.getElementById(`s${s}_p${p}_reg`).value = dr[0];
        document.getElementById(`s${s}_p${p}_type`).value = dr[1];
        document.getElementById(`s${s}_p${p}_order`).value = dr[2];
        document.getElementById(`s${s}_p${p}_name`).value = regsToString(dr.slice(3, 13));
      }
      log(`从机 ${s+1} 配置读取完成`, 'ok');
    }
    toast('全部配置读取完成', 'success');
  } catch(e) {
    log(`读取失败: ${e.message}`, 'err');
    toast('读取失败: ' + e.message, 'error');
  }
}

// ═══════════════════════════════════════
//  写入全部配置
// ═══════════════════════════════════════
async function writeAll() {
  const addr = parseInt(document.getElementById('mb-addr').value);
  try {
    log('开始写入全部配置...', 'wrn');

    // 系统寄存器 (0x0000~0x000A)
    const baud2 = parseInt(document.getElementById('uart2_baudrate').value);
    const baud1 = parseInt(document.getElementById('uart1_baudrate').value);
    const sysVals = [
      baud2 & 0xFFFF, (baud2 >> 16) & 0xFFFF,
      parseInt(document.getElementById('uart2_parity').value),
      parseInt(document.getElementById('uart2_stopbits').value),
      parseInt(document.getElementById('rs485_de_delay').value),
      parseInt(document.getElementById('rs485_re_delay').value),
      parseInt(document.getElementById('local_mb_addr').value),
      parseInt(document.getElementById('slave_count').value),
      parseInt(document.getElementById('report_format').value),
      baud1 & 0xFFFF, (baud1 >> 16) & 0xFFFF,
    ];
    await api('/api/write-multi', {addr, start: 0x0000, values: sysVals});
    log('系统寄存器写入完成', 'ok');

    // 从机配置
    for (let s = 0; s < SLAVE_COUNT; s++) {
      const base = 0x0100 + s * 0x0200;
      const poll = parseInt(document.getElementById(`s${s}_poll`).value);
      const nameStr = document.getElementById(`s${s}_name`).value;
      const nameRegs = stringToRegs(nameStr);

      // 基本配置 (5) + 名称 (10)
      const slaveVals = [
        parseInt(document.getElementById(`s${s}_addr`).value),
        parseInt(document.getElementById(`s${s}_enabled`).value),
        parseInt(document.getElementById(`s${s}_dp_count`).value),
        poll & 0xFFFF, (poll >> 16) & 0xFFFF,
        ...nameRegs
      ];
      await api('/api/write-multi', {addr, start: base, values: slaveVals});

      // 数据点
      for (let p = 0; p < DP_COUNT; p++) {
        const dpBase = base + 0x0010 + p * 0x0010;
        const dpNameStr = document.getElementById(`s${s}_p${p}_name`).value;
        const dpNameRegs = stringToRegs(dpNameStr);
        const dpVals = [
          parseInt(document.getElementById(`s${s}_p${p}_reg`).value),
          parseInt(document.getElementById(`s${s}_p${p}_type`).value),
          parseInt(document.getElementById(`s${s}_p${p}_order`).value),
          ...dpNameRegs
        ];
        await api('/api/write-multi', {addr, start: dpBase, values: dpVals});
      }
      log(`从机 ${s+1} 配置写入完成`, 'ok');
    }
    toast('全部配置写入完成', 'success');
  } catch(e) {
    log(`写入失败: ${e.message}`, 'err');
    toast('写入失败: ' + e.message, 'error');
  }
}

// ═══════════════════════════════════════
//  名称 ↔ 寄存器转换
// ═══════════════════════════════════════
function regsToString(regs) {
  const bytes = [];
  for (const r of regs) {
    bytes.push((r >> 8) & 0xFF);
    bytes.push(r & 0xFF);
  }
  let s = '';
  for (const b of bytes) {
    if (b === 0) break;
    s += String.fromCharCode(b);
  }
  return s;
}

function stringToRegs(str) {
  const regs = [];
  for (let i = 0; i < 20; i += 2) {
    const hi = i < str.length ? str.charCodeAt(i) : 0;
    const lo = (i+1) < str.length ? str.charCodeAt(i+1) : 0;
    regs.push((hi << 8) | lo);
  }
  return regs.slice(0, 10);
}

// ═══════════════════════════════════════
//  JSON 导入导出
// ═══════════════════════════════════════
function getFormConfig() {
  const cfg = {
    system: {
      uart2_baudrate: parseInt(document.getElementById('uart2_baudrate').value),
      uart2_parity: parseInt(document.getElementById('uart2_parity').value),
      uart2_stopbits: parseInt(document.getElementById('uart2_stopbits').value),
      rs485_de_delay: parseInt(document.getElementById('rs485_de_delay').value),
      rs485_re_delay: parseInt(document.getElementById('rs485_re_delay').value),
      local_mb_addr: parseInt(document.getElementById('local_mb_addr').value),
      slave_count: parseInt(document.getElementById('slave_count').value),
      report_format: parseInt(document.getElementById('report_format').value),
      uart1_baudrate: parseInt(document.getElementById('uart1_baudrate').value),
    },
    slaves: []
  };
  for (let s = 0; s < SLAVE_COUNT; s++) {
    const slave = {
      addr: parseInt(document.getElementById(`s${s}_addr`).value),
      enabled: parseInt(document.getElementById(`s${s}_enabled`).value),
      data_point_count: parseInt(document.getElementById(`s${s}_dp_count`).value),
      poll_period_ms: parseInt(document.getElementById(`s${s}_poll`).value),
      name: document.getElementById(`s${s}_name`).value,
      data_points: []
    };
    for (let p = 0; p < DP_COUNT; p++) {
      slave.data_points.push({
        reg_addr: parseInt(document.getElementById(`s${s}_p${p}_reg`).value),
        data_type: parseInt(document.getElementById(`s${s}_p${p}_type`).value),
        byte_order: parseInt(document.getElementById(`s${s}_p${p}_order`).value),
        name: document.getElementById(`s${s}_p${p}_name`).value,
      });
    }
    cfg.slaves.push(slave);
  }
  return cfg;
}

function setFormConfig(cfg) {
  if (cfg.system) {
    const s = cfg.system;
    if (s.uart2_baudrate !== undefined) document.getElementById('uart2_baudrate').value = s.uart2_baudrate;
    if (s.uart2_parity !== undefined) document.getElementById('uart2_parity').value = s.uart2_parity;
    if (s.uart2_stopbits !== undefined) document.getElementById('uart2_stopbits').value = s.uart2_stopbits;
    if (s.rs485_de_delay !== undefined) document.getElementById('rs485_de_delay').value = s.rs485_de_delay;
    if (s.rs485_re_delay !== undefined) document.getElementById('rs485_re_delay').value = s.rs485_re_delay;
    if (s.local_mb_addr !== undefined) document.getElementById('local_mb_addr').value = s.local_mb_addr;
    if (s.slave_count !== undefined) document.getElementById('slave_count').value = s.slave_count;
    if (s.report_format !== undefined) document.getElementById('report_format').value = s.report_format;
    if (s.uart1_baudrate !== undefined) document.getElementById('uart1_baudrate').value = s.uart1_baudrate;
  }
  if (cfg.slaves) {
    for (let s = 0; s < cfg.slaves.length && s < SLAVE_COUNT; s++) {
      const sl = cfg.slaves[s];
      if (sl.addr !== undefined) document.getElementById(`s${s}_addr`).value = sl.addr;
      if (sl.enabled !== undefined) document.getElementById(`s${s}_enabled`).value = sl.enabled;
      if (sl.data_point_count !== undefined) document.getElementById(`s${s}_dp_count`).value = sl.data_point_count;
      if (sl.poll_period_ms !== undefined) document.getElementById(`s${s}_poll`).value = sl.poll_period_ms;
      if (sl.name !== undefined) document.getElementById(`s${s}_name`).value = sl.name;
      if (sl.data_points) {
        for (let p = 0; p < sl.data_points.length && p < DP_COUNT; p++) {
          const dp = sl.data_points[p];
          if (dp.reg_addr !== undefined) document.getElementById(`s${s}_p${p}_reg`).value = dp.reg_addr;
          if (dp.data_type !== undefined) document.getElementById(`s${s}_p${p}_type`).value = dp.data_type;
          if (dp.byte_order !== undefined) document.getElementById(`s${s}_p${p}_order`).value = dp.byte_order;
          if (dp.name !== undefined) document.getElementById(`s${s}_p${p}_name`).value = dp.name;
        }
      }
    }
  }
}

function exportJSON() {
  const cfg = getFormConfig();
  const json = JSON.stringify(cfg, null, 2);
  document.getElementById('json-area').value = json;

  // 同时下载文件
  const blob = new Blob([json], {type: 'application/json'});
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = `stm32l051_config_${new Date().toISOString().slice(0,10)}.json`;
  a.click();
  URL.revokeObjectURL(url);
  toast('配置已导出', 'success');
}

function importJSON() {
  document.getElementById('file-import').click();
}

function handleImport(event) {
  const file = event.target.files[0];
  if (!file) return;
  const reader = new FileReader();
  reader.onload = function(e) {
    try {
      const cfg = JSON.parse(e.target.result);
      setFormConfig(cfg);
      document.getElementById('json-area').value = JSON.stringify(cfg, null, 2);
      toast('配置已导入', 'success');
      log('从 JSON 文件导入配置', 'ok');
    } catch(err) {
      toast('JSON 解析失败: ' + err.message, 'error');
    }
  };
  reader.readAsText(file);
  event.target.value = '';
}

// 也可以从文本框导入
document.getElementById('json-area').addEventListener('keydown', function(e) {
  if (e.ctrlKey && e.key === 'Enter') {
    try {
      const cfg = JSON.parse(this.value);
      setFormConfig(cfg);
      toast('从文本框导入配置', 'success');
    } catch(err) {
      toast('JSON 解析失败', 'error');
    }
  }
});

// ═══════════════════════════════════════
//  启动
// ═══════════════════════════════════════
initSlaveTabs();
scanPorts();
setInterval(scanPorts, 5000);
</script>
</body>
</html>'''


# ═══════════════════════════════════════════════════════════════
#  HTTP 请求处理
# ═══════════════════════════════════════════════════════════════

class Handler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass  # 静默 HTTP 日志

    def _json(self, code, data):
        self.send_response(code)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.end_headers()
        self.wfile.write(json.dumps(data, ensure_ascii=False).encode())

    def do_GET(self):
        path = urlparse(self.path).path
        if path == '/' or path == '/index.html':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode())
        elif path == '/api/ports':
            try:
                import serial.tools.list_ports
                ports = [p.device for p in serial.tools.list_ports.comports()]
            except:
                import glob
                ports = sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*') +
                               glob.glob('/dev/tty.usb*') + glob.glob('COM*'))
            self._json(200, {'ports': ports})
        else:
            self._json(404, {'error': 'Not found'})

    def do_POST(self):
        path = urlparse(self.path).path
        length = int(self.headers.get('Content-Length', 0))
        body = json.loads(self.rfile.read(length)) if length else {}

        try:
            if path == '/api/connect':
                mb.connect(body['port'], body.get('baudrate', 9600),
                          body.get('parity', 'N'))
                self._json(200, {'ok': True})

            elif path == '/api/disconnect':
                mb.disconnect()
                self._json(200, {'ok': True})

            elif path == '/api/read':
                regs = mb.read_holding_regs(body['addr'], body['start'], body['count'])
                self._json(200, {'regs': regs})

            elif path == '/api/write-single':
                mb.write_single_reg(body['addr'], body['reg'], body['value'])
                self._json(200, {'ok': True})

            elif path == '/api/write-multi':
                mb.write_multiple_regs(body['addr'], body['start'], body['values'])
                self._json(200, {'ok': True})

            else:
                self._json(404, {'error': 'Not found'})

        except Exception as e:
            self._json(400, {'error': str(e)})


# ═══════════════════════════════════════════════════════════════
#  启动
# ═══════════════════════════════════════════════════════════════

class ThreadedHTTPServer(socketserver.ThreadingMixIn, HTTPServer):
    daemon_threads = True

def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8080
    server = ThreadedHTTPServer(('0.0.0.0', port), Handler)
    print(f"╔══════════════════════════════════════════════╗")
    print(f"║  STM32L051 Modbus 配置上位机                 ║")
    print(f"║  打开浏览器访问: http://localhost:{port}        ║")
    print(f"║  Ctrl+C 退出                                 ║")
    print(f"╚══════════════════════════════════════════════╝")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n已停止")
        mb.disconnect()
        server.shutdown()

if __name__ == '__main__':
    main()
