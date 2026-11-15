const { JSDOM, VirtualConsole } = require('jsdom');
const fs = require('fs');
const URLB = 'http://127.0.0.1:7911/';
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const errors = [];
async function until(fn, what, ms = 8000) { const t = Date.now(); while (Date.now() - t < ms) { const v = fn(); if (v) return v; await sleep(50); } throw new Error('timeout: ' + what); }
(async () => {
  const vc = new VirtualConsole();
  vc.on('jsdomError', (e) => { if (!/Could not load (script|link)|cdnjs/.test(e.message)) errors.push(e.message); });
  vc.on('error', (e) => errors.push(String(e)));
  const dom = await JSDOM.fromURL(URLB, { runScripts: 'dangerously', resources: 'usable', pretendToBeVisual: true, virtualConsole: vc,
    beforeParse(w) { w.fetch = (u, o) => fetch(new URL(u, URLB), o); w.confirm = () => true; w.prompt = () => 'notes/todo.md'; } });
  const w = dom.window, d = w.document;
  const click = (el) => el.dispatchEvent(new w.MouseEvent('click', { bubbles: true }));
  const btn = (root, text) => [...root.querySelectorAll('button')].find((b) => b.textContent.trim().startsWith(text));

  await until(() => d.querySelector('.welcome h1'), 'welcome');
  console.log('1 welcome ok; editor mode:', w.CodeMirror ? 'codemirror' : 'plain fallback');

  click(btn(d, 'New project'));
  await until(() => d.querySelector('#np-name'), 'wizard');
  const name = d.querySelector('#np-name'); name.value = 'UI Test Bot'; name.dispatchEvent(new w.Event('input'));
  if (!d.querySelector('#np-loc').value.endsWith('/ui_test_bot')) throw new Error('location not derived: ' + d.querySelector('#np-loc').value);
  d.querySelector('#np-loc').value = '/tmp/rfproj/ui_test_bot';
  click(d.querySelector('[data-preset="mixed"]'));
  const st = () => d.querySelector('#np-status').textContent;
  await until(() => /SDKs: ros2-jazzy, moveit, nav2, slam-toolbox, px4-1\.16, gazebo, mavsdk\./.test(st()), 'plan status: ' + st());
  const checked = [...d.querySelectorAll('.comp input:checked')].map((c) => c.dataset.id);
  console.log('2 preset mixed -> modules', checked.join(','));
  // ROS-only modules must explain themselves when ROS is taken away
  const sel = d.querySelector('#np-ros'); sel.value = ''; sel.dispatchEvent(new w.Event('change'));
  await until(() => /needs a ROS 2 distro/.test(st()), 'ros requirement message');
  sel.value = 'ros2-humble'; sel.dispatchEvent(new w.Event('change'));
  await until(() => /SDKs: ros2-humble, moveit/.test(st()) && /install step/.test(st()), 'status recovers');
  // untick a module: its SDK disappears from the plan
  const mv = d.querySelector('[data-id="moveit"]'); mv.checked = false; mv.dispatchEvent(new w.Event('change'));
  await until(() => /SDKs: ros2-humble, nav2/.test(st()), 'module untick');
  mv.checked = true; mv.dispatchEvent(new w.Event('change'));
  await until(() => /moveit/.test(st()), 'module retick');
  console.log('3 ROS switch, requirement feedback, module toggle ok');

  click(d.querySelector('#np-create'));
  await until(() => d.querySelector('#tree .node'), 'tree');
  await until(() => d.querySelector('#code') && d.querySelector('#code').value.includes('# UI Test Bot'), 'README opened');
  if (d.querySelector('.modal')) throw new Error('modal still open');
  const opts = [...d.querySelectorAll('#runsel option')].map((o) => o.value);
  if (!opts.includes('Mission: run sequence') || !opts.includes('Nav2 + SLAM: TurtleBot3 simulation')) throw new Error('mixed run configs missing');
  console.log('4 project open; run configs:', opts.length, '| sync badge:', d.querySelector('#syncbtn .badge').textContent, '| notice:', !!d.querySelector('.notice'));

  click(d.querySelector('[data-path="ros2_ws"]'));
  await until(() => d.querySelector('[data-path="ros2_ws/src"]'), 'expand ros2_ws');
  if (d.querySelector('[data-path="src"]')) throw new Error('ROS package should not be at the project root');
  if (!d.querySelector('[data-path="mav_ws"]')) throw new Error('mav_ws missing');
  click(d.querySelector('[data-path="roboforge.json"]'));
  const ta = await until(() => d.querySelector('#code').value.includes('"components"') && d.querySelector('#code'), 'manifest opened');
  const m = JSON.parse(ta.value); m.run.push({ name: 'Say hi', cmd: 'echo hi-from-ui' });
  ta.value = JSON.stringify(m, null, 2); ta.dispatchEvent(new w.Event('input'));
  if (!d.querySelector('.tab.on').textContent.includes('●')) throw new Error('dirty marker missing');
  d.dispatchEvent(new w.KeyboardEvent('keydown', { key: 's', ctrlKey: true, bubbles: true }));
  await until(() => [...d.querySelectorAll('#runsel option')].some((o) => o.value === 'Say hi'), 'manifest save reloads run configs');
  if (!fs.readFileSync('/tmp/rfproj/ui_test_bot/roboforge.json', 'utf8').includes('hi-from-ui')) throw new Error('not written to disk');
  console.log('5 edit + Ctrl+S ok, run config list refreshed');

  d.querySelector('#runsel').value = 'Say hi'; click(d.querySelector('#runbtn'));
  await until(() => /hi-from-ui\n[\s\S]*\[exit 0\]/.test(d.querySelector('#log').textContent), 'run output');
  await until(() => d.querySelector('.jobs .dot.ok'), 'job status dot');
  const cmd = d.querySelector('#cmd'); cmd.value = 'echo console-$((6*7)); false';
  cmd.dispatchEvent(new w.KeyboardEvent('keydown', { key: 'Enter', bubbles: true }));
  await until(() => /console-42[\s\S]*\[exit 1\]/.test(d.querySelector('#log').textContent), 'console output');
  await until(() => d.querySelector('.jobs .dot.bad'), 'failed dot');
  console.log('6 run config + console ok; job tabs:', d.querySelectorAll('.jobs .tab').length);

  // mix modules into the existing project
  click(d.querySelector('#modbtn'));
  await until(() => d.querySelector('#md-apply'), 'modules dialog');
  const tick = (id, on) => { const cb = d.querySelector('.modal [data-id="' + id + '"]'); cb.checked = on; cb.dispatchEvent(new w.Event('change')); };
  if (!d.querySelector('.modal [data-id="nav2"]').checked) throw new Error('current modules not pre-ticked');
  tick('mujoco', true); tick('moveit', false);
  await until(() => /mujoco/.test(d.querySelector('#md-status').textContent) && !/moveit/.test(d.querySelector('#md-status').textContent), 'modules plan');
  click(d.querySelector('#md-apply'));
  await until(() => d.querySelector('.toast.good'), 'module change toast');
  const man = JSON.parse(fs.readFileSync('/tmp/rfproj/ui_test_bot/roboforge.json', 'utf8'));
  if (!man.modules.includes('mujoco') || man.modules.includes('moveit')) throw new Error('modules not applied: ' + man.modules);
  if (!man.run.some((r) => r.name === 'Say hi') || man.run.some((r) => r.name.startsWith('MoveIt'))) throw new Error('run configs not merged');
  if (!fs.existsSync('/tmp/rfproj/ui_test_bot/ros2_ws/src/ui_test_bot/ui_test_bot/mujoco_bridge.py')) throw new Error('module file missing');
  const setup = fs.readFileSync('/tmp/rfproj/ui_test_bot/ros2_ws/src/ui_test_bot/setup.py', 'utf8');
  if (!setup.includes('mujoco_bridge = ') || !setup.includes('mission = ')) throw new Error('nodes not registered');
  await until(() => d.querySelector('#modbtn .badge').textContent === String(man.modules.length), 'header badge');
  click(d.querySelector('.tabs .tab[title="roboforge.json"]'));
  await until(() => d.querySelector('#code').value.includes('"mujoco"'), 'open manifest tab refreshed from disk');
  if (!man.run.some((r) => r.name === 'MAVSDK: offboard demo' && r.workspace === 'mav')) throw new Error('mav workspace run config');
  console.log('7 modules dialog ok ->', man.modules.join(','), '|', d.querySelector('.toast.good').textContent.split('\n')[1]);

  // workspaces dialog: add a second ROS workspace, then a package in it
  click(d.querySelector('#wsbtn'));
  await until(() => d.querySelector('#ws-add'), 'ws dialog');
  click(d.querySelector('#ws-add'));
  const rowsIn = d.querySelectorAll('.modal .row input[aria-label="Path"]');
  const last = rowsIn[rowsIn.length - 1]; last.value = 'drivers_ws'; last.dispatchEvent(new w.Event('input'));
  click(d.querySelector('#ws-apply'));
  await until(() => d.querySelector('#wsbtn .badge').textContent === String(man.workspaces.length + 1), 'ws badge');
  if (!fs.existsSync('/tmp/rfproj/ui_test_bot/drivers_ws')) throw new Error('drivers_ws not created');
  click(d.querySelector('#pkgbtn'));
  await until(() => d.querySelector('#pk-create'), 'pkg dialog');
  d.querySelector('#pk-name').value = 'Lidar Driver'; d.querySelector('#pk-kind').value = 'ament_cmake';
  const wsOpts = [...d.querySelectorAll('#pk-ws option')]; d.querySelector('#pk-ws').value = wsOpts[wsOpts.length - 1].value;
  click(d.querySelector('#pk-create'));
  await until(() => fs.existsSync('/tmp/rfproj/ui_test_bot/drivers_ws/src/lidar_driver/CMakeLists.txt'), 'package files');
  const act = fs.readFileSync('/tmp/rfproj/ui_test_bot/.roboforge/activate.sh', 'utf8');
  if (!act.includes('"$RF_PROJECT/ros2_ws" "$RF_PROJECT/drivers_ws"') || !act.includes('export ROS_DOMAIN_ID="0"')) throw new Error('activation env wrong');
  console.log('8 workspaces + new package ok');

  click(d.querySelector('[aria-label="New file"]'));
  await until(() => d.querySelector('[data-path="notes/todo.md"]'), 'new file in tree');
  click(btn(d.querySelector('header'), 'SDK Manager'));
  await until(() => d.querySelector('.modal .comp'), 'sdk manager');
  console.log('9 new file + SDK manager ok; catalog rows:', d.querySelectorAll('.modal .comp').length);
  await sleep(300);
  if (errors.length) { console.log('PAGE ERRORS:', errors); process.exit(1); }
  console.log('ALL UI CHECKS PASSED');
  process.exit(0);
})().catch((e) => { console.error('FAIL:', e.message, errors); process.exit(1); });
