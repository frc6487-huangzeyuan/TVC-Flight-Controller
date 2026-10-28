import * as preview from '../web/dist/preview.mjs';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
import {webcrypto} from 'node:crypto';
import * as protocol from '../web/dist/protocol.mjs';

const app=fs.readFileSync(new URL('../web/dist/app.mjs',import.meta.url),'utf8');
const html=fs.readFileSync(new URL('../web/dist/index.html',import.meta.url),'utf8');
const flush=async()=>{for(let i=0;i<20;i++)await Promise.resolve();};
const sleep=ms=>new Promise(r=>setTimeout(r,ms));
function browser(mode='MANUAL'){
  const elements=new Map([...html.matchAll(/id="([^"]+)"/g)].map(m=>[m[1],{
    value:m[1]==='motor-speed'?'25':'135',checked:true,textContent:'',className:'',events:{},
    getBoundingClientRect(){return {width:0};},classList:{toggle(){}},addEventListener(name,fn){this.events[name]=fn;}
  }]));
  const clock={now:10000},intervals=[],events={},windowEvents={};
  const ctx={...protocol,...preview,console,TextEncoder,TextDecoder,crypto:webcrypto,performance:{now:()=>clock.now},
    document:{hidden:false,getElementById(id){assert.ok(elements.has(id),id);return elements.get(id);},addEventListener(n,f){events[n]=f;}},
    window:{addEventListener(n,f){windowEvents[n]=f;}},navigator:{},location:{hostname:'test.invalid'},WebSocket:{OPEN:1},
    setTimeout,clearTimeout,setInterval(fn){intervals.push(fn);}};
  vm.createContext(ctx);
  vm.runInContext(app.replace(/^import[^\n]*\n/gm,'')+'\nglobalThis.api={S,enableServos,startMotor,startAutomatic,stopAll,stopMotor,disableServos,command,rawWrite,closeTransport,acceptPacket,render,recalibrate,switchDeviceMode,toggleAutoMotor,startAutoMotor,sendAutoMotor,stopAutomatic,disconnect,toggleTiltThrottle};',ctx);
  const api=ctx.api,S=api.S,d=new protocol.DemoDevice(S.session);d.operatingMode=mode;d.worldReady=true;d.referenceId=1;
  S.mode='wifi';S.boot=d.boot;S.tele=d.telemetry();S.tele.control.preposition_active=false;S.lastRx=clock.now;S.epoch=d.epoch;
  const sent=[];let onSend;
  S.ws={readyState:1,send(line){const p=protocol.decodeFrame(line.trimEnd());sent.push(p);onSend?.(p);},close(){this.readyState=3;}};
  function telemetry(){const p=d.telemetry();p.control.preposition_active=false;api.acceptPacket(p);return p;}
  function ack(p){const a=d.command(p);api.acceptPacket(a);return a;}
  return {api,S,d,ctx,events,windowEvents,elements,clock,intervals,sent,ack,telemetry,set onSend(fn){onSend=fn;}};
}
let checks=0;
async function test(name,fn){await fn();console.log('PASS '+name);++checks;}
for(const kind of ['servo','auto'])await test('Actual app cancels '+kind+' start before heartbeat ACK after stop',async()=>{
  const b=browser(kind==='auto'?'ATTITUDE':'MANUAL');
  const start=(kind==='auto'?b.api.startAutomatic():b.api.enableServos()).catch(e=>e);
  assert.equal(b.sent[0].cmd,'heartbeat');
  const stop=b.api.stopAll();assert.equal(b.sent[1].cmd,'stop');
  b.ack(b.sent[0]);await flush();assert.deepEqual(b.sent.map(p=>p.cmd),['heartbeat','stop']);
  b.ack(b.sent[1]);b.telemetry();await stop;assert.equal((await start).name,'AbortError');
  assert.equal(b.d.servo,false);assert.equal(b.S.heartbeat,false);await b.api.closeTransport();
});
await test('Actual motor arming wait is canceled by stop and cannot send throttle',async()=>{
  const b=browser();b.onSend=p=>queueMicrotask(()=>{b.ack(p);b.telemetry();});
  const start=b.api.startMotor().catch(e=>e);await flush();
  assert.ok(b.sent.some(p=>p.cmd==='esc_arm'));assert.equal(b.d.armed,false);
  await b.api.stopAll();await sleep(60);assert.equal((await start).name,'AbortError');
  assert.equal(b.sent.some(p=>p.cmd==='esc'),false);assert.equal(b.d.servo,false);await b.api.closeTransport();
});
await test('Hidden page cancels a start even before output telemetry becomes active',async()=>{
  const b=browser();const start=b.api.enableServos().catch(e=>e);
  b.ctx.document.hidden=true;b.events.visibilitychange();b.ack(b.sent[0]);await flush();
  assert.equal((await start).name,'AbortError');assert.deepEqual(b.sent.map(p=>p.cmd),['heartbeat','stop']);
  b.ack(b.sent[1]);b.telemetry();await flush();await b.api.closeTransport();
});
await test('Disconnect cancels pending starts and rejects packets from old session',async()=>{
  const b=browser();const old=b.S.session,start=b.api.enableServos().catch(e=>e);await b.api.closeTransport();
  b.ack(b.sent[0]);await flush();assert.equal((await start).name,'AbortError');assert.equal(b.sent.length,1);
  assert.equal(b.S.mode,null);assert.equal(b.S.tele,null);assert.equal(b.S.session,old);
});
await test('Repeated start clicks produce only one operation',async()=>{
  const b=browser(),first=b.api.enableServos().catch(e=>e),second=await b.api.enableServos().catch(e=>e);
  assert.equal(second.name,'AbortError');assert.equal(b.sent.length,1);await b.api.closeTransport();await first;
});
await test('Idle calibration renews session with keepalive, never actuator heartbeat',async()=>{
  const b=browser('ATTITUDE');b.S.tele.calibrating=true;
  b.intervals[0]();assert.equal(b.sent[0].cmd,'keepalive');assert.equal(b.S.heartbeat,false);
  b.ack(b.sent[0]);await flush();b.clock.now+=1000;b.telemetry();b.intervals[0]();
  assert.equal(b.sent[1].cmd,'keepalive');b.ack(b.sent[1]);await b.api.closeTransport();
});
await test('Serial write rejection does not poison subsequent writes or reconnect',async()=>{
  const b=browser();b.S.mode='serial';b.S.ws=null;
  b.S.writer={write(){return Promise.reject(new Error('injected write failure'));},releaseLock(){}};
  await assert.rejects(b.api.rawWrite('first'),/injected/);await b.api.closeTransport();
  let writes=0;b.S.mode='serial';b.S.writer={write(){++writes;return Promise.resolve();},releaseLock(){}};
  await b.api.rawWrite('new connection');assert.equal(writes,1);await b.api.closeTransport();
});
await test('Old queued serial frames cannot migrate to a replacement connection',async()=>{
  const b=browser();let release,oldCalls=0,newCalls=0;
  b.S.mode='serial';b.S.writer={write(){++oldCalls;return new Promise(r=>{release=r;});},releaseLock(){}};
  const first=b.api.rawWrite('first');await flush();const queued=b.api.rawWrite('queued').catch(e=>e);
  await b.api.closeTransport();b.S.mode='serial';b.S.writer={write(){++newCalls;return Promise.resolve();},releaseLock(){}};
  release();await first;assert.equal((await queued).name,'AbortError');assert.equal(oldCalls,1);assert.equal(newCalls,0);
  await b.api.closeTransport();
});
await test('Serial command timestamp uses telemetry available when queue drains',async()=>{
  const b=browser();let release,frame;
  b.S.mode='serial';let count=0;b.S.writer={write(bytes){if(++count===1)return new Promise(r=>{release=r;});frame=protocol.decodeFrame(new TextDecoder().decode(bytes).trimEnd());queueMicrotask(()=>b.ack(frame));return Promise.resolve();},releaseLock(){}};
  const blocked=b.api.rawWrite('blocking');await flush();const heartbeat=b.api.command('heartbeat');
  b.S.tele.t_us=7000000;b.S.lastRx=b.clock.now;b.clock.now+=100;
  release();await blocked;await heartbeat;assert.equal(frame.device_t_us,7100000);await b.api.closeTransport();
});
await test('Primary stale transport clears status, angles and sample frequency',async()=>{
  const b=browser();b.clock.now+=2501;b.api.render();
  assert.equal(b.elements.get('imu-status').textContent,'连接中断');
  assert.equal(b.elements.get('pitch-value').textContent,'—°');assert.equal(b.elements.get('sample-rate').textContent,'— Hz');
  await b.api.closeTransport();
});
await test('Calibration fault displays sample and read diagnostics, and cancels start',async()=>{
  const b=browser(),start=b.api.enableServos().catch(e=>e),p=b.d.telemetry();
  p.fault='CALIBRATION_DATA';Object.assign(p.control_imu,{calibration_samples:187,calibration_rejected:4,read_errors:12,race_drops:8});
  b.api.acceptPacket(p);assert.match(b.elements.get('notice').textContent,/187\/300/);
  assert.match(b.elements.get('notice').textContent,/读取错误累计 12/);assert.equal((await start).name,'AbortError');assert.equal(b.S.heartbeat,false);
  await b.api.closeTransport();
});
await test('Normal servo and motor commands serialize epoch updates and independent stops',async()=>{
  const b=browser();b.onSend=p=>queueMicrotask(()=>{const ack=b.ack(p);assert.equal(ack.ok,true,p.cmd+': '+ack.reason);b.telemetry();});
  await b.api.enableServos();assert.equal(b.d.servo,true);
  b.d.config={stop_us:1500,min_us:1300,max_us:1700,hold_ms:3000,hz:50};b.d.signal=true;b.d.armed=true;b.telemetry();
  await Promise.all([b.api.command('servo',{x:1600,y:1500}),b.api.command('esc',{us:1550})]);
  assert.equal(b.d.x,1600);assert.equal(b.d.us,1550);
  await b.api.stopMotor();assert.equal(b.d.servo,true);assert.equal(b.d.armed,false);assert.equal(b.S.heartbeat,true);
  await b.api.disableServos();assert.equal(b.S.heartbeat,false);await b.api.closeTransport();
});
await test('Mode switch stays busy until matching device telemetry arrives',async()=>{
  const b=browser('ATTITUDE'),switching=b.api.switchDeviceMode('manual');
  await flush();assert.equal(b.sent[0].cmd,'set_mode');b.ack(b.sent[0]);await flush();
  assert.equal(b.S.operation.kind,'mode');assert.equal(b.elements.get('tab-auto').disabled,true);
  b.telemetry();await switching;assert.equal(b.S.tele.operating_mode,'MANUAL');assert.equal(b.S.operation,null);
  await b.api.closeTransport();
});

await test('Automatic mode defaults motor opt-in off and rejects fan start',async()=>{
  const b=browser('ATTITUDE');b.api.render();
  assert.equal(b.S.autoFanWanted,false);assert.equal(b.d.autoMotorAllowed,false);
  assert.equal(b.elements.get('auto-motor-enable').checked,false);
  assert.equal(b.elements.get('auto-motor-start').disabled,true);
  assert.match((await b.api.startAutoMotor().catch(e=>e)).message,/勾选/);
  assert.equal(b.sent.length,0);await b.api.closeTransport();
});
function autoAck(b,finishArming=false){b.onSend=p=>queueMicrotask(()=>{
  const ack=b.ack(p);assert.equal(ack.ok,true,p.cmd+': '+ack.reason);
  if(finishArming&&p.cmd==='esc_arm')b.d.armingAt=performance.now()-1;
  b.telemetry();
});}
async function optIn(b){b.elements.get('auto-motor-enable').checked=true;await b.api.toggleAutoMotor();}
await test('Old firmware without preposition support cannot start an automatic fan',async()=>{
  const b=browser('ATTITUDE');autoAck(b);await optIn(b);delete b.S.tele.control.preposition_active;
  await assert.rejects(b.api.startAutoMotor(),/不支持电机启动前预定位/);
  assert.equal(b.sent.some(p=>p.cmd==='esc_arm'),false);await b.api.closeTransport();
});
await test('Checkbox grants permission only, retains zero speed and never arms ESC',async()=>{
  const b=browser('ATTITUDE');autoAck(b);await optIn(b);
  assert.deepEqual(b.sent.map(p=>p.cmd),['heartbeat','auto_motor_enable']);
  assert.equal(b.d.autoMotorAllowed,true);assert.equal(b.d.signal,false);assert.equal(b.d.armed,false);
  assert.equal(b.elements.get('auto-motor-speed').value,0);await b.api.closeTransport();
});
await test('Fan runs under ATTITUDE and PID can start while fan stays armed',async()=>{
  const b=browser('ATTITUDE');autoAck(b,true);await optIn(b);await b.api.startAutoMotor();
  assert.equal(b.d.armed,true);assert.equal(b.d.us,1500);assert.equal(b.d.auto,false);
  b.elements.get('auto-motor-speed').value=40;await b.api.sendAutoMotor();assert.equal(b.d.us,1580);
  await b.api.startAutomatic();assert.equal(b.d.auto,true);assert.equal(b.d.servo,true);assert.equal(b.d.armed,true);
  await b.api.stopMotor();assert.equal(b.d.auto,true);assert.equal(b.d.servo,true);assert.equal(b.d.armed,false);
  assert.equal(b.S.autoFanWanted,true);await b.api.closeTransport();
});
await test('Unchecking during ESC arming cancels pending throttle but keeps PID running',async()=>{
  const b=browser('ATTITUDE');autoAck(b);await b.api.startAutomatic();await optIn(b);
  const starting=b.api.startAutoMotor().catch(e=>e);await flush();
  assert.ok(b.d.armingAt);b.elements.get('auto-motor-enable').checked=false;await b.api.toggleAutoMotor();
  await sleep(60);assert.equal((await starting).name,'AbortError');
  assert.equal(b.d.autoMotorAllowed,false);assert.equal(b.d.armingAt,0);assert.equal(b.d.armed,false);
  assert.equal(b.d.auto,true);assert.equal(b.sent.some(p=>p.cmd==='esc'),false);await b.api.closeTransport();
});
await test('Stopping automatic control stops both outputs and revokes fan permission',async()=>{
  const b=browser('ATTITUDE');autoAck(b,true);await optIn(b);await b.api.startAutoMotor();await b.api.startAutomatic();
  await b.api.stopAutomatic();assert.equal(b.d.auto,false);assert.equal(b.d.servo,false);assert.equal(b.d.armed,false);
  assert.equal(b.d.autoMotorAllowed,false);assert.equal(b.S.autoFanWanted,false);
  assert.equal(b.elements.get('auto-motor-enable').checked,false);await b.api.closeTransport();
});
await test('Unchecking before permission heartbeat ACK cannot later enable the fan',async()=>{
  const b=browser('ATTITUDE');b.elements.get('auto-motor-enable').checked=true;
  const enabling=b.api.toggleAutoMotor().catch(e=>e);assert.equal(b.sent[0].cmd,'heartbeat');
  b.elements.get('auto-motor-enable').checked=false;const disabling=b.api.toggleAutoMotor();
  b.ack(b.sent[0]);b.ack(b.sent[1]);b.telemetry();await disabling;await flush();
  assert.equal((await enabling).name,'AbortError');assert.deepEqual(b.sent.map(p=>p.cmd),['heartbeat','auto_motor_disable']);
  assert.equal(b.d.autoMotorAllowed,false);await b.api.closeTransport();
});
await test('Hidden page clears automatic fan opt-in and stops armed motor',async()=>{
  const b=browser('ATTITUDE');autoAck(b,true);await optIn(b);await b.api.startAutoMotor();
  b.ctx.document.hidden=true;b.events.visibilitychange();await flush();await sleep(60);
  assert.equal(b.S.autoFanWanted,false);assert.equal(b.d.autoMotorAllowed,false);assert.equal(b.d.armed,false);await b.api.closeTransport();
});
await test('Switching to manual revokes automatic fan permission and preserves mode separation',async()=>{
  const b=browser('ATTITUDE');autoAck(b,true);await optIn(b);await b.api.startAutoMotor();
  await b.api.switchDeviceMode('manual');assert.equal(b.d.operatingMode,'MANUAL');assert.equal(b.d.armed,false);
  assert.equal(b.d.autoMotorAllowed,false);assert.equal(b.S.autoFanWanted,false);await b.api.closeTransport();
});

await test('Disconnect immediately clears opt-in and locks controls before stop ACK',async()=>{
  const b=browser('ATTITUDE');autoAck(b);await optIn(b);b.onSend=null;
  const closing=b.api.disconnect();assert.equal(b.S.autoFanWanted,false);assert.equal(b.S.stopping,1);
  assert.equal(b.elements.get('auto-motor-start').disabled,true);assert.equal(b.elements.get('auto-motor-enable').disabled,true);
  b.ack(b.sent.at(-1));b.telemetry();await closing;assert.equal(b.S.mode,null);assert.equal(b.S.stopping,0);
});
function selectTiltThrottle(b){
  const fields={'throttle-base':0,'throttle-gain':1,'throttle-max':30,'throttle-deadband':2,'throttle-ramp':10};
  for(const [id,value] of Object.entries(fields))b.elements.get(id).value=value;
  b.elements.get('tilt-throttle-enable').checked=true;return b.api.toggleTiltThrottle();
}
await test('Tilt throttle opt-in alone sends no command and keeps outputs stopped',async()=>{
  const b=browser('ATTITUDE');await selectTiltThrottle(b);
  assert.equal(b.sent.length,0);assert.equal(b.d.armed,false);assert.equal(b.d.tiltThrottle.enabled,false);
  await b.api.closeTransport();
});
await test('Coupled start arms at stop before enabling PID and tilt throttle',async()=>{
  const b=browser('ATTITUDE');autoAck(b,true);await selectTiltThrottle(b);await b.api.startAutomatic();
  assert.deepEqual(b.sent.map(p=>p.cmd),['heartbeat','tilt_throttle_config','auto_motor_enable','esc_config','esc_arm','auto_enable']);
  assert.equal(b.d.auto,true);assert.equal(b.d.armed,true);assert.equal(b.d.tiltThrottle.enabled,true);assert.equal(b.d.us,1500);
  await b.api.stopMotor();assert.equal(b.d.auto,true);assert.equal(b.d.tiltThrottle.enabled,false);assert.equal(b.d.armed,false);
  await b.api.closeTransport();
});
await test('Cancel tilt opt-in during arming cannot later enable automatic throttle',async()=>{
  const b=browser('ATTITUDE');autoAck(b);await selectTiltThrottle(b);const starting=b.api.startAutomatic().catch(e=>e);await flush();
  for(let n=0;n<20&&!b.d.armingAt;n++)await flush();assert(b.d.armingAt);b.elements.get('tilt-throttle-enable').checked=false;await b.api.toggleTiltThrottle();await sleep(60);
  assert.equal((await starting).name,'AbortError');assert.equal(b.d.armed,false);assert.equal(b.d.armingAt,0);
  assert.equal(b.sent.some(p=>p.cmd==='auto_enable'),false);assert.equal(b.d.tiltThrottle.enabled,false);await b.api.closeTransport();
});
await test('Full stop revokes coupled throttle and independent motor permission',async()=>{
  const b=browser('ATTITUDE');autoAck(b,true);await selectTiltThrottle(b);await b.api.startAutomatic();await b.api.stopAutomatic();
  assert.equal(b.d.servo,false);assert.equal(b.d.auto,false);assert.equal(b.d.armed,false);assert.equal(b.d.autoMotorAllowed,false);
  assert.equal(b.S.autoThrottleWanted,false);assert.equal(b.elements.get('tilt-throttle-enable').checked,false);await b.api.closeTransport();
});
await test('Invalid tilt throttle settings cannot send arming commands',async()=>{
  const b=browser('ATTITUDE');await selectTiltThrottle(b);b.elements.get('throttle-base').value=50;
  await assert.rejects(b.api.startAutomatic(),/自动油门参数/);assert.equal(b.sent.length,0);await b.api.closeTransport();
});
await test('Missing heartbeat ACK does not block the next 200ms heartbeat',async()=>{
  const b=browser();b.S.heartbeat=true;b.d.servo=true;b.telemetry();
  b.intervals[0]();assert.equal(b.sent[0].cmd,'heartbeat');
  b.clock.now+=200;b.telemetry();b.intervals[0]();assert.equal(b.sent[1].cmd,'heartbeat');
  assert.equal(b.S.heartbeat,true);for(const request of b.sent)b.ack(request);await flush();await b.api.closeTransport();
});
await test('Heartbeat warning retains pending control commands and running outputs',async()=>{
  const b=browser();b.S.heartbeat=true;b.d.servo=true;b.telemetry();
  const changing=b.api.command('servo',{x:1600,y:1500});await flush();
  const action=b.S.action,p=b.d.telemetry();p.warning='HEARTBEAT_TIMEOUT';p.heartbeat_age_ms=1200;
  b.api.acceptPacket(p);assert.equal(b.S.heartbeat,true);assert.equal(b.S.action,action);assert.equal(b.S.pending.size,1);
  assert.equal(b.d.servo,true);assert.equal(b.elements.get('servo-x').disabled,false);
  assert.equal(b.elements.get('device-fault-code').textContent,'HEARTBEAT_TIMEOUT');
  assert.match(b.elements.get('device-fault-description').textContent,/不停止输出/);
  b.ack(b.sent[0]);await changing;assert.equal(b.d.x,1600);await b.api.closeTransport();
});
await test('Heartbeat warning also preserves coupled automatic throttle and PID',async()=>{
  const b=browser('ATTITUDE');autoAck(b,true);await selectTiltThrottle(b);await b.api.startAutomatic();
  const action=b.S.action,p=b.d.telemetry();p.warning='HEARTBEAT_TIMEOUT';p.heartbeat_age_ms=1200;b.api.acceptPacket(p);
  assert.equal(b.S.action,action);assert.equal(b.S.heartbeat,true);assert.equal(b.S.autoThrottleWanted,true);
  assert.equal(b.d.auto,true);assert.equal(b.d.servo,true);assert.equal(b.d.armed,true);assert.equal(b.d.tiltThrottle.enabled,true);
  await b.api.stopAll();await b.api.closeTransport();
});
console.log(`${checks} actual-app lifecycle regressions passed; no device connected.`);
