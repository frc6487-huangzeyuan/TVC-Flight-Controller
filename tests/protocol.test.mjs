import assert from 'node:assert/strict';
import {VERSION,crc32,encodeFrame,decodeFrame,LineDecoder,isTelemetry,canOperate,controlLeaseValid,telemetryToCsv,validateEscConfig,DemoDevice,isFreshCommandTime} from '../web/dist/protocol.mjs';
let checks=0;
function check(name,fn){fn();checks++;console.log('PASS '+name);}
function calibratedDemo(){
  const device=new DemoDevice('test1234');
  assert.ok(device.command({epoch:device.epoch,cmd:'calibrate',seq:1}).ok);
  // 模拟同一个三秒校准窗口完成，由网页轮询推动时间前进。
  device.calibrateUntil=performance.now()-1;
  assert.equal(device.telemetry().world.valid,true);
  assert.ok(device.command({epoch:device.epoch,cmd:'set_mode',mode:'MANUAL',seq:2}).ok);
  return device;
}
function withClock(start,run){
  const descriptor=Object.getOwnPropertyDescriptor(globalThis,'performance');
  let now=start;
  Object.defineProperty(globalThis,'performance',{configurable:true,value:{now:()=>now}});
  try{return run({advance:delta=>{now+=delta;}});}
  finally{Object.defineProperty(globalThis,'performance',descriptor);}
}
function csvRecord(packet){
  const lines=telemetryToCsv([{...packet,mode:'demo',received:'2026-10-03T00:00:00Z'}]).trim().split('\r\n');
  const headers=lines[0].replace(/^\ufeff/,'').split(',');
  const values=lines[1].split(',').map(value=>value.startsWith('"')?value.slice(1,-1).replaceAll('""','"'):value);
  return Object.fromEntries(headers.map((key,index)=>[key,values[index]]));
}
check('CRC32 IEEE known vector',()=>assert.equal(crc32('123456789'),'cbf43926'));
const packet={v:7,seq:1,type:'command',session:'test1234',cmd:'hello',note:'中文😀'};
check('UTF8 exact-byte round trip',()=>{assert.equal(VERSION,7);assert.deepEqual(decodeFrame(encodeFrame(packet).trimEnd()),packet);});
check('CRC corrupted frame rejected',()=>assert.throws(()=>decodeFrame(encodeFrame(packet).replace('hello','jello').trimEnd()),/CRC/));
check('Unsupported version rejected',()=>{for(const v of [1,2,3,4,5,6,8])assert.throws(()=>decodeFrame(encodeFrame({...packet,v}).trimEnd()),/VERSION/);});
check('Chunked byte stream and combined frames',()=>{const d=new LineDecoder(),bytes=new TextEncoder().encode(encodeFrame(packet)+encodeFrame({...packet,seq:2})),out=[];for(let i=0;i<bytes.length;i++)d.push(bytes.slice(i,i+1),p=>out.push(p),e=>{throw e;});assert.equal(out.length,2);assert.equal(out[0].note,packet.note);});
check('Oversized frame safely resynchronizes',()=>{const d=new LineDecoder(),out=[],errors=[];d.push('x'.repeat(9000)+'\n'+encodeFrame(packet),p=>out.push(p),e=>errors.push(e));assert.equal(errors.length,1);assert.equal(out.length,1);});
check('Unknown ESC has no usable output',()=>{const d=calibratedDemo(),a=d.command({epoch:d.epoch,cmd:'esc_arm',seq:2,config_rev:0});assert.equal(a.ok,false);assert.equal(d.telemetry().outputs.esc_armed,false);assert.equal(d.telemetry().outputs.esc_us,0);});
const config={stop_us:1500,min_us:1400,max_us:1600,hold_ms:1000,hz:50,confirmed:true};
check('ESC bidirectional range and confirmation validated',()=>{assert.deepEqual(validateEscConfig({...config}),config);assert.throws(()=>validateEscConfig({...config,min_us:1500}));assert.throws(()=>validateEscConfig({...config,max_us:1500}));assert.throws(()=>validateEscConfig({...config,confirmed:false}));assert.throws(()=>validateEscConfig({...config,hz:400}));assert.throws(()=>validateEscConfig({...config,max_us:2200}));});
check('Freshness, fault and sensor gates are independent of world calibration',()=>{const p=calibratedDemo().telemetry();assert.ok(isTelemetry(p));assert.ok(canOperate(p,20));assert.equal(canOperate(p,800),false);assert.equal(canOperate({...p,fault:'IMU_TIMEOUT'},20),false);assert.equal(canOperate({...p,imus:[{...p.imus[0],valid:false}]},20),false);assert.equal(canOperate({...p,imus:[{...p.imus[0],a:[0,null,1]}]},20),false);assert.equal(canOperate({...p,world:{...p.world,valid:false},calibrated:false},20),true);assert.equal('estop_ok' in p,false);});
check('Attitude and manual modes are isolated',()=>{
  const d=new DemoDevice('test1234');assert.equal(d.telemetry().operating_mode,'ATTITUDE');
  assert.ok(d.command({epoch:d.epoch,cmd:'calibrate',seq:1}).ok);d.calibrateUntil=performance.now()-1;
  const detected=d.telemetry();assert.equal(detected.world.valid,true);assert.equal(canOperate(detected,0),false);
  assert.equal(d.command({epoch:d.epoch,cmd:'servo_enable',seq:2}).ok,false);
  assert.ok(d.command({epoch:d.epoch,cmd:'set_mode',mode:'MANUAL',seq:3}).ok);assert.ok(canOperate(d.telemetry(),0));
  assert.ok(d.command({epoch:d.epoch,cmd:'servo_enable',seq:4}).ok);assert.equal(d.telemetry().outputs.servo_enabled,true);
  assert.ok(d.command({epoch:d.epoch,cmd:'set_mode',mode:'ATTITUDE',seq:5}).ok);const locked=d.telemetry();
  assert.equal(locked.operating_mode,'ATTITUDE');assert.equal(locked.outputs.servo_enabled,false);assert.equal(locked.calibrating,true);assert.equal(canOperate(locked,0),false);
  assert.equal(isTelemetry({...locked,operating_mode:'AUTO'}),false);
});
check('Manual mode clears only world calibration faults',()=>{
  for(const fault of ['CALIBRATION_MOVED','CALIBRATION_DATA','HEADING_UNDEFINED']){
    const d=new DemoDevice('test1234');d.fault=fault;
    assert.ok(d.command({epoch:d.epoch,cmd:'set_mode',mode:'MANUAL',seq:1}).ok);assert.equal(d.telemetry().fault,'');
  }
  const d=new DemoDevice('test1234');d.fault='IMU_TIMEOUT';
  assert.ok(d.command({epoch:d.epoch,cmd:'set_mode',mode:'MANUAL',seq:1}).ok);assert.equal(d.telemetry().fault,'IMU_TIMEOUT');
});
check('Old sample cannot claim valid',()=>{const p=calibratedDemo().telemetry();assert.equal(canOperate({...p,t_us:10000000,imus:p.imus.map(s=>({...s,sample_t_us:0}))},20),false);assert.equal(canOperate({...p,calibrating:undefined},20),false);assert.equal(canOperate({...p,state:'UNKNOWN'},20),false);});
check('ESC config alone does not start its signal',()=>{const d=new DemoDevice('test1234');assert.ok(d.command({epoch:d.epoch,cmd:'set_mode',mode:'MANUAL',seq:1}).ok);assert.ok(d.command({epoch:d.epoch,cmd:'esc_config',seq:2,...config}).ok);assert.equal(d.telemetry().outputs.esc_signal_enabled,false);assert.equal(d.telemetry().outputs.esc_us,0);});
check('Stop invalidates prior servo epoch',()=>{const d=calibratedDemo();assert.ok(d.command({epoch:d.epoch,cmd:'servo_enable',seq:2}).ok);const old=d.epoch;d.stop();assert.equal(d.command({epoch:d.epoch,cmd:'servo',seq:3,epoch:old,x:1550,y:1500}).ok,false);assert.equal(d.telemetry().outputs.servo_enabled,false);});
check('270-degree servo accepts 500 to 2500 us only',()=>{const d=calibratedDemo();assert.ok(d.command({epoch:d.epoch,cmd:'servo_enable',seq:2}).ok);for(const [seq,x] of [[3,500],[4,1500],[5,2500]])assert.ok(d.command({epoch:d.epoch,cmd:'servo',seq,epoch:d.epoch,x,y:1500}).ok);for(const [seq,x] of [[6,499],[7,2501]])assert.equal(d.command({epoch:d.epoch,cmd:'servo',seq,epoch:d.epoch,x,y:1500}).ok,false);});
check('Manual servos and motor can run together and stop independently',()=>{
  const d=calibratedDemo();assert.ok(d.command({epoch:d.epoch,cmd:'servo_enable',seq:2}).ok);
  assert.ok(d.command({epoch:d.epoch,cmd:'esc_config',seq:3,...config}).ok);assert.equal(d.telemetry().outputs.servo_enabled,true);
  assert.ok(d.command({epoch:d.epoch,cmd:'esc_arm',seq:4,config_rev:d.rev}).ok);d.armingAt=performance.now()-1;
  let p=d.telemetry();assert.equal(p.outputs.servo_enabled,true);assert.equal(p.outputs.esc_armed,true);
  assert.ok(d.command({epoch:d.epoch,cmd:'servo_disable',seq:5}).ok);p=d.telemetry();assert.equal(p.outputs.servo_enabled,false);assert.equal(p.outputs.esc_armed,true);
  assert.ok(d.command({epoch:d.epoch,cmd:'servo_enable',seq:6}).ok);assert.ok(d.command({epoch:d.epoch,cmd:'esc_stop',seq:7}).ok);
  p=d.telemetry();assert.equal(p.outputs.servo_enabled,true);assert.equal(p.outputs.esc_armed,false);
});
check('Automatic return requires a world reference and stays isolated from manual motor control',()=>{
  const d=new DemoDevice('test1234');assert.equal(d.command({epoch:d.epoch,cmd:'auto_enable',seq:1}).ok,false);
  assert.ok(d.command({epoch:d.epoch,cmd:'calibrate',seq:2}).ok);d.calibrateUntil=performance.now()-1;assert.equal(d.telemetry().world.valid,true);
  assert.ok(d.command({epoch:d.epoch,cmd:'auto_enable',seq:3}).ok);let p=d.telemetry();assert.equal(p.control.auto_enabled,true);assert.equal(p.outputs.servo_enabled,true);
  assert.equal(d.command({epoch:d.epoch,cmd:'esc_config',seq:4,...config}).ok,false);
  assert.ok(d.command({epoch:d.epoch,cmd:'auto_disable',seq:5}).ok);p=d.telemetry();assert.equal(p.control.auto_enabled,false);assert.equal(p.outputs.servo_enabled,false);
});
check('ESC arming hold, bidirectional range and rearming',()=>{const d=calibratedDemo();assert.ok(d.command({epoch:d.epoch,cmd:'esc_config',seq:2,...config}).ok);assert.ok(d.command({epoch:d.epoch,cmd:'esc_arm',seq:3,config_rev:d.rev}).ok);assert.equal(d.command({epoch:d.epoch,cmd:'esc',seq:4,epoch:d.epoch,us:1450}).ok,false);d.armingAt=performance.now()-1;assert.equal(d.telemetry().outputs.esc_armed,true);assert.equal(d.command({epoch:d.epoch,cmd:'esc',seq:5,epoch:d.epoch,us:1300}).ok,false);assert.ok(d.command({epoch:d.epoch,cmd:'esc',seq:6,epoch:d.epoch,us:1450}).ok);assert.ok(d.command({epoch:d.epoch,cmd:'esc',seq:7,epoch:d.epoch,us:1550}).ok);d.stop();assert.equal(d.telemetry().outputs.esc_us,1500);assert.equal(d.command({epoch:d.epoch,cmd:'esc',seq:8,epoch:d.epoch,us:1520}).ok,false);});
check('Stop, hidden page and new session cancel pending control lease',()=>{const start={generation:2,session:'A'},current={generation:2,session:'A',heartbeatEnabled:true,hidden:false};assert.equal(controlLeaseValid(start,current),true);for(const patch of [{generation:3},{session:'B'},{heartbeatEnabled:false},{hidden:true}])assert.equal(controlLeaseValid(start,{...current,...patch}),false);});
for(const cancel of ['stop','hidden']){let release;const ack=new Promise(resolve=>release=resolve),start={generation:2,session:'A'},current={...start,heartbeatEnabled:true,hidden:false};let enabled=false;const delayed=async()=>{await ack;if(controlLeaseValid(start,current))enabled=true;};const pending=delayed();if(cancel==='stop'){current.generation++;current.heartbeatEnabled=false;}else current.hidden=true;release();await pending;assert.equal(enabled,false);checks++;console.log('PASS Delayed heartbeat ACK cannot enable after '+cancel);}
check('Exactly one MPU6050 at 0x68 required',()=>{const p=new DemoDevice('test1234').telemetry();assert.equal(p.imu_count,1);assert.equal(p.imus.length,1);assert.ok(isTelemetry(p));assert.equal(isTelemetry({...p,imus:[...p.imus,...p.imus]}),false);assert.equal(isTelemetry({...p,imus:[{...p.imus[0],address:0x69}]}),false);assert.equal(isTelemetry({...p,imu_count:2}),false);});
check('CSV contains one sensor, mode and simulation label',()=>{const p=new DemoDevice('test1234').telemetry(),csv=telemetryToCsv([{...p,mode:'demo',received:'2026-10-03T00:00:00Z'}]),lines=csv.trim().split('\r\n');assert.equal(lines.length,2);assert.match(lines[0],/operating_mode/);assert.doesNotMatch(lines[0],/estop_ok/);assert.match(lines[0],/imu1_ax_g/);assert.doesNotMatch(lines[0],/imu2_/);assert.equal(lines[0].split(',').length,49);assert.equal(lines[1].split(',').length,49);assert.match(lines[1],/^"demo",/);});
check('Uncalibrated manual mode can enable servo or configured ESC',()=>{
  const d=new DemoDevice('test1234');
  assert.ok(d.command({epoch:d.epoch,cmd:'set_mode',mode:'MANUAL',seq:1}).ok);
  assert.equal(canOperate(d.telemetry(),0),true);
  assert.ok(d.command({epoch:d.epoch,cmd:'servo_enable',seq:2}).ok);
  d.stop();
  assert.ok(d.command({epoch:d.epoch,cmd:'esc_config',seq:3,...config}).ok);
  assert.ok(d.command({epoch:d.epoch,cmd:'esc_arm',seq:4,config_rev:d.rev}).ok);
  const p=d.telemetry();
  assert.equal(p.calibrated,false);assert.equal(p.world.valid,false);
  assert.equal(p.outputs.servo_enabled,false);assert.equal(p.outputs.esc_signal_enabled,true);
  assert.equal(p.outputs.esc_arming,true);assert.equal(p.outputs.esc_us,1500);
});
check('Initial null world vectors can be received and manual mode authorizes control',()=>{
  const d=new DemoDevice('test1234'),initial=d.telemetry();
  const p=initial;
  assert.equal(p.world.reference_id,0);assert.equal(p.world.heading_absolute,false);
  assert.deepEqual(p.world.f_world_g,[null,null,null]);
  assert.deepEqual(p.world.linear_world_mps2,[null,null,null]);
  assert.ok(isTelemetry(p));assert.equal(canOperate(p,0),false);
  assert.equal(isTelemetry({...p,world:{...p.world,valid:true}}),false);
  assert.ok(d.command({epoch:d.epoch,cmd:'set_mode',mode:'MANUAL',seq:1}).ok);
  assert.equal(canOperate(d.telemetry(),0),true);
});
check('Calibration requires the full window before output becomes available',()=>withClock(10000,clock=>{
  const d=new DemoDevice('test1234');assert.ok(d.command({epoch:d.epoch,cmd:'set_mode',mode:'MANUAL',seq:1}).ok);assert.ok(d.command({epoch:d.epoch,cmd:'calibrate',seq:2}).ok);
  clock.advance(2999);const waiting=d.telemetry();
  assert.equal(waiting.calibrating,true);assert.equal(waiting.world.valid,false);
  assert.equal(canOperate(waiting,0),false);assert.equal(d.command({epoch:d.epoch,cmd:'servo_enable',seq:3}).ok,false);
  clock.advance(1);const ready=d.telemetry();
  assert.equal(ready.calibrating,false);assert.equal(ready.calibrated,true);
  assert.equal(ready.world.valid,true);assert.equal(ready.world.reference_id,1);
  assert.ok(canOperate(ready,0));assert.ok(d.command({epoch:d.epoch,cmd:'servo_enable',seq:4}).ok);
}));
check('World zero preserves tilt and changes the explicit reference and epoch',()=>withClock(10000,clock=>{
  const d=calibratedDemo();clock.advance(5000);const before=d.telemetry(),epoch=d.epoch;
  assert.ok(Math.abs(before.world.euler_deg[2])>1);
  assert.ok(d.command({epoch:d.epoch,cmd:'world_zero',seq:2}).ok);const after=d.telemetry();
  assert.equal(d.epoch,epoch+1);assert.equal(after.world.reference_id,before.world.reference_id+1);
  assert.equal(after.world.reference_age_s,0);assert.equal(after.world.heading_absolute,false);
  assert.deepEqual(after.world.euler_deg.slice(0,2),before.world.euler_deg.slice(0,2));
  assert.ok(Math.abs(after.world.euler_deg[0])+Math.abs(after.world.euler_deg[1])>1);
  assert.ok(Math.abs(after.world.euler_deg[2])<1e-12);
  for(let i=6;i<9;i++)assert.ok(Math.abs(after.world.r_bw[i]-before.world.r_bw[i])<1e-12);
  assert.ok(isTelemetry(after));
}));
check('World zero is forbidden before calibration or while any output is active',()=>{
  const fresh=new DemoDevice('test1234');assert.equal(fresh.command({epoch:fresh.epoch,cmd:'world_zero',seq:1}).ok,false);
  const servo=calibratedDemo(),reference=servo.referenceId;
  assert.ok(servo.command({epoch:servo.epoch,cmd:'servo_enable',seq:2}).ok);
  assert.equal(servo.command({epoch:servo.epoch,cmd:'world_zero',seq:3}).ok,false);assert.equal(servo.referenceId,reference);
  const esc=calibratedDemo();assert.ok(esc.command({epoch:esc.epoch,cmd:'esc_config',seq:2,...config}).ok);
  assert.ok(esc.command({epoch:esc.epoch,cmd:'esc_arm',seq:3,config_rev:esc.rev}).ok);
  assert.equal(esc.command({epoch:esc.epoch,cmd:'world_zero',seq:4}).ok,false);
  esc.armingAt=performance.now()-1;assert.equal(esc.telemetry().outputs.esc_armed,true);
  assert.equal(esc.command({epoch:esc.epoch,cmd:'world_zero',seq:5}).ok,false);
});
check('Invalid quaternion, nonfinite world data and wrong world frame are rejected',()=>{
  const p=calibratedDemo().telemetry();
  const patches=[{q_bw:[0,0,0,0]},{q_bw:[2,0,0,0]},{q_bw:[NaN,0,0,1]},{q_bw:[1,0,0]},
    {f_world_g:[NaN,0,1]},{linear_world_mps2:[0,Infinity,0]},{euler_deg:[0,NaN,0]},
    {r_bw:[NaN,...p.world.r_bw.slice(1)]},{frame:'NED'},{heading_absolute:true},
    {reference_id:0},{reference_age_s:-1},{valid:false,f_world_g:[null,NaN,null]}];
  for(const patch of patches){const invalid={...p,world:{...p.world,...patch}};assert.equal(isTelemetry(invalid),false);assert.equal(canOperate(invalid,0),false);}
  assert.equal(isTelemetry({...p,calibrated:false}),false);
  assert.ok(isTelemetry({...p,world:{...p.world,q_bw:p.world.q_bw.map(n=>-n)}}));
});
check('Command time rejects stale, far future and invalid timestamps',()=>{
  const now=1000000;
  assert.ok(isFreshCommandTime(now,now));assert.ok(isFreshCommandTime(now-500000,now));
  assert.equal(isFreshCommandTime(now-500001,now),false);
  assert.ok(isFreshCommandTime(now+100000,now));assert.equal(isFreshCommandTime(now+100001,now),false);
  for(const sent of [-1,NaN,Infinity,1.5,'1000000',Number.MAX_SAFE_INTEGER+1])assert.equal(isFreshCommandTime(sent,now),false);
});
check('CSV world columns preserve validity, reference, yaw and null values',()=>withClock(10000,clock=>{
  const initial=csvRecord(new DemoDevice('test1234').telemetry());
  assert.equal(initial.world_valid,'false');assert.equal(initial.world_reference_id,'0');
  assert.equal(initial.heading_absolute,'false');
  for(const key of ['world_fx_g','world_fy_g','world_fz_g','world_ax_mps2','world_ay_mps2','world_az_mps2'])assert.equal(initial[key],'');
  const d=calibratedDemo();clock.advance(2500);const p=d.telemetry(),row=csvRecord(p);
  assert.equal(row.world_valid,'true');assert.equal(Number(row.world_reference_id),p.world.reference_id);
  assert.equal(row.world_reference_age_s,'2.5');assert.equal(row.heading_absolute,'false');
  assert.equal(row.gravity_correction,String(p.world.correction_active));
  for(const [index,key] of ['world_roll_deg','world_pitch_deg','world_yaw_deg'].entries())assert.equal(Number(row[key]),p.world.euler_deg[index]);
  for(const [index,key] of ['qw','qx','qy','qz'].entries())assert.equal(Number(row[key]),p.world.q_bw[index]);
  assert.equal(row.world_fz_g,'1');assert.equal(row.world_az_mps2,'0');
  assert.ok(d.command({epoch:d.epoch,cmd:'world_zero',seq:2}).ok);const reset=csvRecord(d.telemetry());
  assert.equal(Number(reset.world_reference_id),p.world.reference_id+1);
  assert.equal(reset.world_reference_age_s,'0');assert.equal(reset.world_yaw_deg,'0');
}));

check('New start requires an explicit current epoch after stop',()=>{
  const d=calibratedDemo(),old=d.epoch;d.command({cmd:'stop',seq:10});
  for(const epoch of [undefined,old]){
    const ack=d.command({cmd:'servo_enable',seq:11,epoch});
    assert.equal(ack.ok,false);assert.equal(ack.reason,'OLD_EPOCH');assert.equal(d.servo,false);
  }
  assert.equal(d.command({cmd:'servo_enable',seq:12,epoch:d.epoch}).ok,true);
});
console.log(`${checks} protocol and safety checks passed. Hardware tests remain pending.`);
