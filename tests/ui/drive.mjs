// Minimal headless-Chrome driver for UI checks, with no dependencies.
//   SCRATCH=<dir> [URL=http://127.0.0.1:7979/] node tests/ui/drive.mjs <steps.mjs>
// <steps.mjs> default-exports `async ({ ev, shot, sleep, send }) => { ... }`:
//   ev(js)      run JS in the page (an async function body) and return its value
//   shot(name)  save a screenshot to $SCRATCH/<name>.png
//   send(m, p)  raw Chrome DevTools Protocol call
import { spawn } from 'node:child_process';
import fs from 'node:fs';
import path from 'node:path';

const S = process.env.SCRATCH || '/tmp';
fs.mkdirSync(S, { recursive: true });
const chrome = spawn(process.env.CHROME || '/usr/bin/google-chrome',
  ['--headless=new', '--remote-debugging-port=9333', `--user-data-dir=${S}/chrome`, '--window-size=1500,950', '--no-first-run', '--disable-gpu', 'about:blank'], { stdio: 'ignore' });
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
let ws, id = 0;
const pending = new Map(), logs = [];

async function connect() {
  for (let i = 0; i < 60; i++) {
    try {
      const pages = await (await fetch('http://127.0.0.1:9333/json')).json();
      const pg = pages.find((t) => t.type === 'page');
      if (pg) { ws = new WebSocket(pg.webSocketDebuggerUrl); await new Promise((r, j) => { ws.onopen = r; ws.onerror = j; }); break; }
    } catch (e) { /* not up yet */ }
    await sleep(250);
  }
  ws.onmessage = (m) => {
    const d = JSON.parse(m.data);
    if (d.id && pending.has(d.id)) { pending.get(d.id)(d); pending.delete(d.id); }
    else if (d.method === 'Runtime.exceptionThrown') logs.push('EXC ' + JSON.stringify(d.params.exceptionDetails).slice(0, 400));
    else if (d.method === 'Runtime.consoleAPICalled' && d.params.type === 'error') logs.push('console.error ' + d.params.args.map((a) => a.value || a.description).join(' '));
  };
}
const send = (method, params = {}) => new Promise((r) => { const i = ++id; pending.set(i, r); ws.send(JSON.stringify({ id: i, method, params })); });
const ev = async (expr) => {
  const r = await send('Runtime.evaluate', { expression: `(async()=>{${expr}})()`, awaitPromise: true, returnByValue: true });
  if (r.result.exceptionDetails) throw new Error(JSON.stringify(r.result.exceptionDetails).slice(0, 600));
  return r.result.result.value;
};
const shot = async (name) => { const r = await send('Page.captureScreenshot', { format: 'png' }); fs.writeFileSync(`${S}/${name}.png`, Buffer.from(r.result.data, 'base64')); };

try {
  await connect();
  await send('Runtime.enable'); await send('Page.enable');
  await send('Page.navigate', { url: process.env.URL || 'http://127.0.0.1:7979/' }); await sleep(2500);
  const steps = (await import(path.resolve(process.argv[2]))).default;
  await steps({ ev, shot, sleep, send });
} catch (e) { console.log('DRIVER ERROR', e.message); }
finally {
  console.log(logs.length ? 'PAGE ERRORS:\n' + logs.join('\n') : 'no page errors');
  chrome.kill(); process.exit(0);
}
