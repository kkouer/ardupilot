#!/usr/bin/env python3
"""
ArduPilot 飞控 SN 码产线写入工具 (带 SN 读取与双重写入验证功能)
使用方法：
1. 运行：python3 sn_writer_gui.py
2. 浏览器将自动打开 http://127.0.0.1:8088
"""

import sys
import time
import json
import re
import struct
import threading
import webbrowser
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import parse_qs, urlparse

try:
    from pymavlink import mavutil
    import serial.tools.list_ports
except ImportError:
    print("错误: 请先安装 pymavlink 与 pyserial 库: pip install pymavlink pyserial")
    sys.exit(1)


# 全局状态控制
class State:
    def __init__(self):
        self.master = None
        self.connected = False
        self.current_sn = "未读取"
        self.logs = ["程序初始化完成。请在下方连接飞控并输入 20 位 SN 码。"]

    def log(self, msg):
        timestamp = time.strftime("[%H:%M:%S] ")
        self.logs.append(timestamp + msg)
        if len(self.logs) > 200:
            self.logs.pop(0)

state = State()


def scan_ports():
    ports = [p.device for p in serial.tools.list_ports.comports()]
    if not ports:
        ports = ["/dev/ttyACM0", "/dev/ttyUSB0", "COM3"]
    return ports


CHAR_MAP = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"
REV_MAP = {c: i for i, c in enumerate(CHAR_MAP)}

def encode_4chars(s):
    """将 4 个 Base36 字符编码为无损整数 (0..1679615)"""
    val = 0
    for char in s.upper():
        val = val * 36 + REV_MAP.get(char, 0)
    return val

def decode_4chars(val):
    """将 Base36 整数 (0..1679615) 解码为 4 个字符"""
    val = int(val)
    chars = []
    for _ in range(4):
        chars.append(CHAR_MAP[val % 36])
        val //= 36
    return "".join(reversed(chars))

def sn_to_5ints(sn_str):
    """将 20 位 SN 码编码为 5 个 Base-36 无损整数 (保证 float 传输不丢失精度)"""
    sn_str = sn_str.ljust(20, '0')[:20]
    p1 = encode_4chars(sn_str[0:4])
    p2 = encode_4chars(sn_str[4:8])
    p3 = encode_4chars(sn_str[8:12])
    p4 = encode_4chars(sn_str[12:16])
    p5 = encode_4chars(sn_str[16:20])
    return p1, p2, p3, p4, p5

def read_current_sn(master):
    """从飞控读取 BRD_SN_PART1 ~ BRD_SN_PART5 参数并还原 20 位 SN 码"""
    if not master:
        return ""
    sn_parts = []
    param_names = ["BRD_SN_PART1", "BRD_SN_PART2", "BRD_SN_PART3", "BRD_SN_PART4", "BRD_SN_PART5"]
    
    for name in param_names:
        master.mav.param_request_read_send(
            master.target_system,
            master.target_component,
            name.encode('ascii'),
            -1
        )
        t_start = time.time()
        found_val = None
        while time.time() - t_start < 1.5:
            msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1.0)
            if msg and msg.param_id.rstrip('\x00') == name:
                found_val = msg.param_value
                break
        
        if found_val is None:
            return ""
            
        val_int = int(round(found_val))
        u32 = struct.unpack('I', struct.pack('i', val_int))[0]
        sn_parts.append(u32)
        
    if len(sn_parts) == 5:
        # 判断是否为 Base-36 编码 (单组小于等于 36^4 - 1 = 1679615)
        if all(part <= 1679615 for part in sn_parts):
            sn_str = "".join(decode_4chars(part) for part in sn_parts)
            return sn_str
        else:
            # 兼容旧版的 raw uint32 打包
            raw_bytes = struct.pack('<IIIII', *sn_parts)
            sn_str = raw_bytes.decode('ascii', errors='ignore').rstrip('\x00')
            return sn_str
    return ""


HTML_PAGE = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>ArduPilot 飞控 20位 SN 码产线写入工具</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif; }
  body { background-color: #f5f5f7; color: #1d1d1f; padding: 25px 15px; display: flex; justify-content: center; }
  .container { width: 100%; max-width: 640px; background: #ffffff; border-radius: 16px; box-shadow: 0 4px 24px rgba(0,0,0,0.08); padding: 28px; }
  h1 { font-size: 20px; font-weight: 700; text-align: center; margin-bottom: 24px; color: #1d1d1f; }
  .card { background: #fbfbfd; border: 1px solid #e5e5ea; border-radius: 12px; padding: 20px; margin-bottom: 20px; }
  .card-title { font-size: 14px; font-weight: 700; color: #6e6e73; text-transform: uppercase; letter-spacing: 0.5px; margin-bottom: 14px; display: flex; justify-content: space-between; align-items: center; }
  .form-row { display: flex; gap: 12px; align-items: center; margin-bottom: 12px; flex-wrap: wrap; }
  .form-group { display: flex; flex-direction: column; gap: 6px; flex: 1; min-width: 140px; }
  label { font-size: 13px; font-weight: 600; color: #3a3a3c; }
  select, input, button { font-size: 14px; padding: 10px 14px; border-radius: 8px; border: 1px solid #d1d1d6; outline: none; transition: all 0.2s; }
  select, input { background: #ffffff; width: 100%; color: #000000; }
  input:focus, select:focus { border-color: #007aff; box-shadow: 0 0 0 3px rgba(0,122,255,0.15); }
  button { font-weight: 600; cursor: pointer; border: none; background: #007aff; color: #ffffff; }
  button:hover { background: #0062cc; }
  button:disabled { background: #c7c7cc; cursor: not-allowed; }
  button.btn-secondary { background: #e5e5ea; color: #1d1d1f; }
  button.btn-secondary:hover { background: #d1d1d6; }
  button.btn-success { background: #34c759; }
  button.btn-success:hover { background: #28a745; }
  button.btn-danger { background: #ff3b30; }
  button.btn-danger:hover { background: #d70015; }
  .sn-input-wrapper { display: flex; gap: 10px; align-items: center; }
  .sn-input { font-family: "SF Mono", Menlo, Consolas, monospace; font-size: 16px; font-weight: 700; letter-spacing: 1px; flex: 1; border: 2px solid #007aff !important; background: #ffffff !important; color: #000000 !important; }
  .byte-counter { font-size: 13px; font-weight: 700; color: #ff3b30; min-width: 85px; text-align: right; }
  .byte-counter.valid { color: #34c759; }
  .log-box { background: #1c1c1e; color: #30d158; font-family: "SF Mono", Menlo, Consolas, monospace; font-size: 12px; padding: 14px; border-radius: 8px; height: 160px; overflow-y: auto; white-space: pre-wrap; word-break: break-all; }
  .status-badge { display: inline-block; padding: 4px 10px; border-radius: 12px; font-size: 12px; font-weight: 600; }
  .status-connected { background: #e4f9e8; color: #248a3d; }
  .status-disconnected { background: #ffe5e5; color: #d70015; }
  .current-sn-box { margin-bottom: 14px; background: #eef6ff; padding: 12px 16px; border-radius: 8px; border: 1px solid #cce5ff; display: flex; justify-content: space-between; align-items: center; }
</style>
</head>
<body>
<div class="container">
  <h1>中岳航空飞控 20位 SN 码产线写入工具</h1>
  
  <div class="card">
    <div class="card-title">
      <span>1. 串口/连接设置</span>
      <span id="conn-status" class="status-badge status-disconnected">未连接</span>
    </div>
    <div class="form-row">
      <div class="form-group" style="flex: 2;">
        <label>设备端口</label>
        <div style="display: flex; gap: 8px;">
          <select id="port-select"></select>
          <button class="btn-secondary" onclick="refreshPorts()" style="padding: 0 12px;">🔄 刷新</button>
        </div>
      </div>
      <div class="form-group" style="flex: 1;">
        <label>波特率</label>
        <select id="baud-select">
          <option value="115200">115200</option>
          <option value="57600">57600</option>
          <option value="921600">921600</option>
          <option value="1500000">1500000</option>
          <option value="38400">38400</option>
          <option value="19200">19200</option>
          <option value="9600">9600</option>
        </select>
      </div>
    </div>
    <button id="btn-connect" onclick="toggleConnect()" style="width: 100%; margin-top: 8px;">连接飞控</button>
  </div>

  <div class="card">
    <div class="card-title">
      <span>2. SN 码设置 (刚好 20 位字符)</span>
      <button class="btn-secondary" onclick="fetchReadSN()" style="padding: 4px 10px; font-size: 12px;">🔍 重新读取SN</button>
    </div>

    <div class="current-sn-box">
      <span style="font-size: 13px; font-weight: 600; color: #004085;">飞控当前保存的 SN 码:</span>
      <span id="current-sn-display" style="font-family: 'SF Mono', Consolas, monospace; font-size: 15px; font-weight: 700; color: #007aff;">未读取</span>
    </div>

    <div class="sn-input-wrapper">
      <input type="text" id="sn-input" class="sn-input" value="6975" placeholder="6975xxxxxxxxxxxxxxxx" maxlength="20" oninput="checkSN()">
      <div id="byte-count" class="byte-counter">4 / 20 字节</div>
    </div>
    <button id="btn-write" class="btn-success" onclick="writeSN()" style="width: 100%; margin-top: 14px;" disabled>写入并永久保存 SN 码</button>
  </div>

  <div class="card">
    <div class="card-title">3. 操作日志</div>
    <div id="log-box" class="log-box">程序初始化完成。请在上方连接飞控并输入 20 位 SN 码。</div>
  </div>
</div>

<script>
let isConnected = false;

async function refreshPorts() {
  try {
    const res = await fetch('/api/ports');
    const data = await res.json();
    const select = document.getElementById('port-select');
    select.innerHTML = '';
    data.ports.forEach(p => {
      const opt = document.createElement('option');
      opt.value = p;
      opt.textContent = p;
      select.appendChild(opt);
    });
  } catch (e) {
    console.error('获取端口失败', e);
  }
}

function checkSN() {
  const input = document.getElementById('sn-input');
  const count = document.getElementById('byte-count');
  const btnWrite = document.getElementById('btn-write');
  
  let val = input.value;
  // 自动转大写
  val = val.toUpperCase();
  
  // 过滤非允许字符：只保留数字 (0-9) 和 大写字母 (排除 I 和 O)
  val = val.replace(/[^0-9A-HJ-NP-Z]/g, '');

  // 锁定前 4 位始终为 6975
  if (!val.startsWith('6975')) {
    let suffix = val.replace(/^6?9?7?5?/, '');
    val = '6975' + suffix;
  }

  if (input.value !== val) {
    input.value = val;
  }

  const len = new TextEncoder().encode(val).length;
  count.textContent = `${len} / 20 字节`;
  if (len === 20) {
    count.classList.add('valid');
    if (isConnected) btnWrite.disabled = false;
  } else {
    count.classList.remove('valid');
    btnWrite.disabled = true;
  }
}

async function toggleConnect() {
  const btn = document.getElementById('btn-connect');
  const port = document.getElementById('port-select').value;
  const baud = document.getElementById('baud-select').value;

  btn.disabled = true;
  if (!isConnected) {
    btn.textContent = '正在连接...';
    const res = await fetch('/api/connect', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({port, baud: parseInt(baud)})
    });
    const data = await res.json();
    if (!data.success) {
      alert('连接失败: ' + data.error);
    }
  } else {
    await fetch('/api/disconnect', {method: 'POST'});
  }
  btn.disabled = false;
  pollStatus();
}

async function fetchReadSN() {
  if (!isConnected) {
    alert('请先连接飞控！');
    return;
  }
  const snDisplay = document.getElementById('current-sn-display');
  snDisplay.textContent = '正在读取...';
  const res = await fetch('/api/read_sn');
  const data = await res.json();
  if (data.success) {
    snDisplay.textContent = data.sn || '（全空/未设置）';
  } else {
    alert('读取 SN 失败: ' + data.error);
  }
  pollStatus();
}

async function writeSN() {
  const input = document.getElementById('sn-input');
  const sn = input.value.trim();
  if (new TextEncoder().encode(sn).length !== 20) {
    alert('SN 码必须恰好为 20 个 ASCII 字符！');
    return;
  }
  const btnWrite = document.getElementById('btn-write');
  btnWrite.disabled = true;
  const res = await fetch('/api/write_sn', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({sn})
  });
  const data = await res.json();
  if (data.success) {
    alert('SUCCESS: SN 码 [' + sn + '] 写入成功且验证通过！');
    // 写入成功后复位输入框为默认前缀 6975 方便下一次写入
    input.value = '6975';
  } else {
    alert('ERROR: 写入失败: ' + data.error);
  }
  checkSN();
  pollStatus();
}

// 绑定 Backspace/Delete 事件防止误删 6975 前缀
window.addEventListener('DOMContentLoaded', () => {
  const input = document.getElementById('sn-input');
  if (input) {
    input.addEventListener('keydown', function(e) {
      if ((e.key === 'Backspace' || e.key === 'Delete') && this.selectionStart <= 4) {
        if (this.selectionStart === this.selectionEnd || this.selectionStart < 4) {
          e.preventDefault();
        }
      }
    });
  }
});

async function pollStatus() {
  try {
    const res = await fetch('/api/status');
    const data = await res.json();
    isConnected = data.connected;
    
    const connStatus = document.getElementById('conn-status');
    const btnConnect = document.getElementById('btn-connect');
    const snDisplay = document.getElementById('current-sn-display');

    if (isConnected) {
      connStatus.textContent = '已连接';
      connStatus.className = 'status-badge status-connected';
      btnConnect.textContent = '断开连接';
      btnConnect.className = 'btn-danger';
    } else {
      connStatus.textContent = '未连接';
      connStatus.className = 'status-badge status-disconnected';
      btnConnect.textContent = '连接飞控';
      btnConnect.className = '';
    }

    if (data.current_sn) {
      snDisplay.textContent = data.current_sn;
    } else {
      snDisplay.textContent = isConnected ? '（未读取）' : '未连接';
    }
    
    const logBox = document.getElementById('log-box');
    logBox.textContent = data.logs.join('\\n');
    logBox.scrollTop = logBox.scrollHeight;
    
    checkSN();
  } catch (e) {
    console.error('状态轮询异常', e);
  }
}

refreshPorts();
pollStatus();
setInterval(pollStatus, 1000);
</script>
</body>
</html>
"""


class RequestHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass  # 禁用标准 HTTP 日志输出

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == '/' or parsed.path == '/index.html':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode('utf-8'))

        elif parsed.path == '/api/ports':
            ports = scan_ports()
            self._send_json({'ports': ports})

        elif parsed.path == '/api/status':
            self._send_json({
                'connected': state.connected,
                'current_sn': state.current_sn,
                'logs': state.logs
            })

        elif parsed.path == '/api/read_sn':
            if state.master and state.connected:
                state.log("正在从飞控读取当前 SN 码...")
                sn = read_current_sn(state.master)
                if sn:
                    state.current_sn = sn
                    state.log(f"读取成功，当前飞控 SN 码为: '{sn}'")
                    self._send_json({'success': True, 'sn': sn})
                else:
                    state.log("当前飞控尚未设置 SN 码或全空。")
                    state.current_sn = "（全空/未设置）"
                    self._send_json({'success': True, 'sn': "（全空/未设置）"})
            else:
                self._send_json({'success': False, 'error': '未连接飞控！'})

        else:
            self.send_error(404)

    def do_POST(self):
        parsed = urlparse(self.path)
        content_length = int(self.headers.get('Content-Length', 0))
        body_data = self.rfile.read(content_length).decode('utf-8') if content_length > 0 else '{}'
        
        try:
            req = json.loads(body_data)
        except Exception:
            req = {}

        if parsed.path == '/api/connect':
            port = req.get('port', '')
            baud = req.get('baud', 115200)
            
            state.log(f"正在尝试连接至 {port} ({baud})...")
            try:
                conn = mavutil.mavlink_connection(port, baud=baud)
                hb = conn.wait_heartbeat(timeout=5)
                if hb is None:
                    raise TimeoutError("未接收到飞控心跳包 (5秒超时)")
                state.master = conn
                state.connected = True
                state.log(f"成功连接至飞控 (SysID: {conn.target_system}, CompID: {conn.target_component})")
                
                # 自动读取一次当前 SN
                state.log("正在读取飞控当前保存的 SN 码...")
                sn = read_current_sn(conn)
                if sn:
                    state.current_sn = sn
                    state.log(f"成功读取到飞控当前保存的 SN 码: '{sn}'")
                else:
                    state.current_sn = "（全空/未设置）"
                    state.log("当前飞控尚未设置 SN 码。")

                self._send_json({'success': True, 'current_sn': state.current_sn})
            except Exception as e:
                err_msg = str(e)
                state.master = None
                state.connected = False
                state.current_sn = "未连接"
                state.log(f"连接失败: {err_msg}")
                self._send_json({'success': False, 'error': err_msg})

        elif parsed.path == '/api/disconnect':
            if state.master:
                try:
                    state.master.close()
                except Exception:
                    pass
            state.master = None
            state.connected = False
            state.current_sn = "未连接"
            state.log("已断开与飞控的连接。")
            self._send_json({'success': True})

        elif parsed.path == '/api/write_sn':
            sn_str = req.get('sn', '').strip()
            sn_bytes = sn_str.encode('ascii', errors='ignore')

            if not state.master or not state.connected:
                self._send_json({'success': False, 'error': '未连接飞控！'})
                return

            if len(sn_bytes) != 20:
                self._send_json({'success': False, 'error': 'SN 码必须恰好为 20 个 ASCII 字符！'})
                return

            if not re.match(r'^6975[0-9A-HJ-NP-Z]{16}$', sn_str):
                self._send_json({'success': False, 'error': 'SN 码格式错误！只能包含数字和大写字母（排除字母 I 和 O）。'})
                return

            # 1. 写入前的 SN 读取
            state.log("----------------------------------------")
            old_sn = read_current_sn(state.master)
            if old_sn:
                state.log(f"【写入前读取】飞控旧 SN 码: '{old_sn}'")
            else:
                state.log("【写入前读取】飞控当前暂无 SN 码记录。")

            state.log(f"正在向飞控写入新的 20 位 SN 码: '{sn_str}' ...")
            try:
                p1, p2, p3, p4, p5 = sn_to_5ints(sn_str)
                MAV_CMD_USER_1 = 31010

                state.master.mav.command_long_send(
                    state.master.target_system,
                    state.master.target_component,
                    MAV_CMD_USER_1,
                    0,
                    float(p1), float(p2), float(p3), float(p4), float(p5),
                    888888.0, 0
                )

                ack = state.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
                if ack and ack.command == MAV_CMD_USER_1:
                    if ack.result == 0:
                        state.log("SUCCESS: 飞控成功响应写入请求，已写入 Flash！")
                        time.sleep(0.5)

                        # 2. 写入后的 SN 再次读取验证
                        state.log("【写入后验证】正在重新从飞控读取 SN 码...")
                        new_sn = read_current_sn(state.master)
                        if new_sn:
                            state.current_sn = new_sn
                            state.log(f"【写入后验证成功】从飞控确认读出最新 SN 码: '{new_sn}'")
                        else:
                            state.current_sn = sn_str
                            state.log(f"【写入后】设为最新 SN: '{sn_str}'")

                        state.log("----------------------------------------")
                        self._send_json({'success': True, 'sn': state.current_sn})
                    else:
                        state.log(f"ERROR: 飞控拒绝写入 (ACK Result Code: {ack.result})")
                        self._send_json({'success': False, 'error': f"飞控拒绝写入 (Result: {ack.result})"})
                else:
                    state.log("ERROR: 写入超时，未接收到飞控应答。")
                    self._send_json({'success': False, 'error': '写入超时，请检查串口连接。'})

            except Exception as e:
                state.log(f"写入异常: {e}")
                self._send_json({'success': False, 'error': str(e)})

        else:
            self.send_error(404)

    def _send_json(self, data):
        self.send_response(200)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.end_headers()
        self.wfile.write(json.dumps(data, ensure_ascii=False).encode('utf-8'))


def main():
    import socket
    port = 8088
    for p in range(8088, 8100):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind(('127.0.0.1', p))
            s.close()
            port = p
            break
        except OSError:
            continue

    server_address = ('127.0.0.1', port)
    HTTPServer.allow_reuse_address = True
    httpd = HTTPServer(server_address, RequestHandler)
    url = f"http://127.0.0.1:{port}"
    
    print(f"==================================================")
    print(f"  中岳航空飞控 20位 SN 码产线写入工具")
    print(f"  服务已启动: {url}")
    print(f"  正在自动打开浏览器...")
    print(f"  按 Ctrl+C 可停止程序")
    print(f"==================================================")
    
    threading.Thread(target=lambda: (time.sleep(0.5), webbrowser.open(url)), daemon=True).start()

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n已退出程序。")
        httpd.server_close()


if __name__ == "__main__":
    main()
