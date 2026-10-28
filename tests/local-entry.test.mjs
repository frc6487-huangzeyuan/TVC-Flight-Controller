import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
import {webcrypto} from 'node:crypto';

const root=new URL('../',import.meta.url);
const html=fs.readFileSync(new URL('web/dist/index.html',root),'utf8');
const bundle=fs.readFileSync(new URL('web/dist/app.bundle.js',root),'utf8');
const standalone=fs.readFileSync(new URL('../TVC控制台.html',root),'utf8');
assert.match(html,/<script defer src="app\.bundle\.js\?v=2\.4\.3"><\/script>/);
assert.doesNotMatch(html,/<script[^>]*type="module"/);
assert.doesNotMatch(standalone,/<(?:script|link)[^>]*(?:src|href)=/);
assert.ok(standalone.includes('<script>'+bundle+'</script>'));
assert.match(standalone,/\.page\[hidden\]\{display:none\}/);

function load(source){
  const elements=new Map([...standalone.matchAll(/id="([^"]+)"/g)].map(m=>[m[1],{
    value:m[1]==='auto-motor-speed'?'0':m[1]==='motor-speed'?'25':'135',
    checked:false,hidden:m[1]==='page-manual',textContent:'',events:{},
    getBoundingClientRect(){return {width:0};},classList:{toggle(){}},addEventListener(n,f){this.events[n]=f;}
  }]));
  let hardwareAttempts=0;
  const ctx={console,TextEncoder,TextDecoder,crypto:webcrypto,performance:{now:()=>10000},
    document:{hidden:false,getElementById(id){assert.ok(elements.has(id),id);return elements.get(id);},addEventListener(){}},
    window:{addEventListener(){}},navigator:{serial:{requestPort(){++hardwareAttempts;throw Error('unexpected hardware request');}}},
    location:{protocol:'file:',hostname:'',href:'file:///test/TVC控制台.html'},
    WebSocket:class {static OPEN=1;constructor(){++hardwareAttempts;throw Error('unexpected hardware request');}},
    fetch(){++hardwareAttempts;throw Error('unexpected hardware request');},
    setInterval(){},setTimeout(){throw Error('unexpected auto connect');},clearTimeout(){}
  };
  vm.createContext(ctx);vm.runInContext(source,ctx);
  assert.equal(elements.get('firmware-version').textContent,'未连接');
  assert.equal(elements.get('connect-wifi').textContent,'打开设备页面');
  assert.equal(elements.get('auto-motor-enable').checked,false);
  assert.equal(elements.get('auto-motor-start').disabled,true);
  elements.get('tab-manual').events.click();
  assert.equal(elements.get('page-auto').hidden,true);
  assert.equal(elements.get('page-manual').hidden,false);
  elements.get('tab-auto').events.click();
  assert.equal(elements.get('page-manual').hidden,true);
  assert.equal(hardwareAttempts,0);
  elements.get('connect-wifi').events.click();
  assert.equal(ctx.location.href,'http://192.168.4.1/?ui=2.4.3');
  assert.equal(hardwareAttempts,0);
}
load(bundle);
load(standalone.match(/<script>([\s\S]*)<\/script>/)[1]);
console.log('PASS classic and standalone local entries load, tabs work, opt-in stays off, version is visible, and file opening never connects to hardware.');
