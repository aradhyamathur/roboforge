// Smoke pass: open an SDK project (PROJECT=<dir>), the Stack view and the New Project picker.
export default async ({ ev, shot, sleep }) => {
  const dir = process.env.PROJECT;
  await sleep(1500);
  await ev(`await openProject(${JSON.stringify(dir)});`);
  await sleep(1500);
  await shot('1-project');
  console.log('title:', await ev(`return document.title + ' | header buttons: ' + [...document.querySelectorAll('header button')].map(b => b.textContent.trim()).join(', ');`));
  await ev(`await stackDialog();`);
  await sleep(2500);
  await shot('2-stack');
  console.log('stack dialog:', (await ev(`const d = [...document.querySelectorAll('dialog,.modal,.dlg')].pop(); return d ? d.innerText.replace(/\\s+/g, ' ').slice(0, 500) : 'NO DIALOG';`)));
  await ev(`document.dispatchEvent(new KeyboardEvent('keydown', {key: 'Escape'})); [...document.querySelectorAll('dialog')].forEach(d => d.close && d.close());`);
  await ev(`newProject();`);
  await sleep(1200);
  console.log('picker:', await ev(`return 'sdk box: ' + !!document.querySelector('#sk-on') + ', quick picks: ' + [...document.querySelectorAll('[data-combo]')].map(b => b.dataset.combo).join(' / ');`));
  await shot('3-new');
};
