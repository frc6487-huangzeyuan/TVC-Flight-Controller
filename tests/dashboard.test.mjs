// 浏览器 DOM 的最小主机替身，仅验证离线状态、主数据源和两种舵机映射。
// 不连接串口，不模拟真实硬件验收。
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import vm from 'node:vm';
import {webcrypto} from 'node:crypto';
import {VERSION,encodeFrame,decodeFrame,LineDecoder,isTelemetry,servoAngleToUs,validatePidSettings} from '../web/dist/protocol.mjs';
const html=readFileSync(new URL('../web/dist/index.html',import.meta.url),'utf8');
const elements=new Map();
for(const match of html.matchAll(/<([\w-]+)\b[^>]*\bid="([^"]+)"[^>]*>/g)){
  const tag=match[0],value=tag.match(/\bvalue="([^"]*)"/)?.[1]??'';
  assert(!elements.has(match[2]),`duplicate ID ${match[2]}`);
  elements.set(match[2],{value,checked:false,disabled:tag.includes('disabled'),textContent:'',hidden:false,
    events:{},classList:{toggle(){}},addEventListener(name,fn){this.events[name]=fn;},closest(){return null;},
    getBoundingClientRect(){return {width:580,height:210};},
    getContext(){return {setTransform(){},clearRect(){},beginPath(){},moveTo(){},lineTo(){},stroke(){},fillText(){}};}});
}
const context=vm.createContext({console,TextEncoder,TextDecoder,crypto:webcrypto,performance,
  setTimeout(){return 1;},clearTimeout(){},setInterval(){return 1;},
  WebSocket:{OPEN:1},location:{hostname:'localhost'},window:{devicePixelRatio:1,addEventListener(){}},
  document:{hidden:false,activeElement:{closest(){return null;}},addEventListener(){},
    getElementById(id){assert(elements.has(id),`missing DOM element ${id}`);return elements.get(id);}}});
vm.runInContext(readFileSync(new URL('../web/dist/app.bundle.js',import.meta.url),'utf8').replace(/^\(function\(\)\{'use strict';/m,"'use strict';").replace(/\}\)\(\);\s*$/,''),context);
assert.equal(elements.get('auto-servo-x').textContent,'—°');
assert.equal(elements.get('apply-pid').disabled,true);
assert.equal(elements.get('chart-state').textContent,'等待真实设备数据');
assert.equal(servoAngleToUs(180,0),1500);assert.equal(servoAngleToUs(135,1),1500);
assert.equal(servoAngleToUs(360,0),2500);assert.equal(servoAngleToUs(85,1),1130);assert.equal(servoAngleToUs(185,1),1870);assert.throws(()=>servoAngleToUs(84,1));assert.throws(()=>servoAngleToUs(186,1));
assert.throws(()=>servoAngleToUs(271,1));
const pid={error_deg:0,p_deg:0,i_deg:0,d_deg:0,output_deg:0,kp:.6,ki:0,kd:.03};
const packet={v:VERSION,type:'telemetry',seq:1,boot:'test-only',session:vm.runInContext('S.session',context),t_us:1000000,
  firmware:'TEST FIXTURE',fault:'',epoch:1,config_rev:0,state:'DISARMED',operating_mode:'ATTITUDE',calibrating:false,calibrated:true,imu_count:1,sample_hz:200,
  world:{frame:'LOCAL_Z_UP',valid:true,heading_absolute:false,heading_defined:true,reference_id:1,correction_active:true,reference_age_s:1,
    euler_deg:[0,0,0],a_body_g:[0,0,1],gyro_body_dps:[0,0,0],f_world_g:[0,0,1],linear_world_mps2:[0,0,0],q_bw:[1,0,0,0],r_bw:[1,0,0,0,1,0,0,0,1]},
  control:{source_id:1,auto_motor_allowed:false,auto_enabled:false,geometry:'AZIMUTH_ELEVATION',mount_ready:true,tilt_valid:true,error_deg:[0,0],azimuth_zero_deg:180,sign_x:1,sign_y:1,pid:{dt_s:.01,output_limit_deg:15,limited:false,x:pid,y:pid}},
  outputs:{servo_enabled:false,servo_us:[1500,1500],servo_range_deg:[360,270],esc_armed:false,esc_arming:false,esc_configured:false,esc_signal_enabled:false,esc_us:0},
  imus:[{address:0x68,source:'SUPERMINI_UART',valid:true,present:true,a:[0,-1,0],g:[0,0,0],pitch:0,roll:0,sample_t_us:995000,temperature:28}],
  imu2:{address:0x69,present:false,valid:false,tilt_valid:false,sample_t_us:0,status:'NOT_FOUND'}};
packet.control_imu={...structuredClone(packet.imus[0]),source_id:1,calibrated:true};
assert(isTelemetry(packet),'primary control packet accepts absent secondary sensor');
const clone=()=>structuredClone(packet);
let bad=clone();bad.imus[0].address=0x69;assert(!isTelemetry(bad),'secondary cannot become primary');
bad=clone();bad.imus[0].source='LOCAL_I2C';assert(!isTelemetry(bad),'local sensor cannot replace SuperMini');
bad=clone();bad.imus[0].sample_t_us=1;assert(!isTelemetry(bad),'stale primary cannot enable controls');
bad=clone();bad.v=7;assert(!isTelemetry(bad),'old protocol cannot use new angle mapping');
let received=[],errors=[];const decoder=new LineDecoder(),frame=new TextEncoder().encode(encodeFrame({...packet,note:'中文校验'}));
for(const byte of frame)decoder.push(new Uint8Array([byte]),p=>received.push(p),e=>errors.push(e));
assert.equal(errors.length,0);assert.equal(received[0].note,'中文校验');
vm.runInContext(`S.mode='serial';acceptPacket(${JSON.stringify(packet)});`,context);
assert.match(elements.get('mounting-state').textContent,/未报告安装偏角/,'old firmware must not show an assumed angle as applied');
const aligned=clone();aligned.control.mount_yaw_deg=-135;
vm.runInContext(`acceptPacket(${JSON.stringify(aligned)});`,context);
assert.match(elements.get('mounting-state').textContent,/IMU1 安装偏角 -135°.*已纳入姿态与 PID/);
aligned.control.mount_ready=false;aligned.calibrating=true;
vm.runInContext(`acceptPacket(${JSON.stringify(aligned)});`,context);
assert.match(elements.get('mounting-state').textContent,/等待竖直校准/);
vm.runInContext(`acceptPacket(${JSON.stringify(packet)});`,context);
assert.equal(elements.get('auto-servo-x').textContent,'180.0°');
assert.equal(elements.get('auto-servo-y').textContent,'135.0°');
assert.equal(elements.get('auto-start').disabled,false);
assert.equal(elements.get('imu2-status').textContent,'未检测到');
assert.equal(elements.get('apply-pid').disabled,false);
const historical=clone();historical.control_imu.sample_timeout_us=2000000;
historical.control_imu.sample_t_us=900000;historical.control_imu.history_state='PREDICTED';
assert(isTelemetry(historical),'primary history survives the previous 50ms threshold');
vm.runInContext(`acceptPacket(${JSON.stringify(historical)});`,context);
assert.equal(elements.get('imu-status').textContent,'历史估算');
assert.equal(elements.get('auto-start').disabled,false);
historical.control_imu.sample_t_us=1;historical.control_imu.history_state='HELD';
vm.runInContext(`acceptPacket(${JSON.stringify(historical)});`,context);
assert.equal(elements.get('imu-status').textContent,'历史保持');
assert.equal(vm.runInContext('S.tele.control_imu.sample_t_us',context),1,'original measurement timestamp is not refreshed');
historical.t_us=3000000;assert(!isTelemetry(historical),'history older than 2s cannot remain valid');
vm.runInContext('S.lastRx=-100000;render();',context);
assert.equal(elements.get('auto-start').disabled,true);
assert.equal(elements.get('auto-servo-x').textContent,'—°');
assert.equal(elements.get('primary-gyro').textContent,'— / — / —');
assert.equal(elements.get('apply-pid').disabled,true);
assert.throws(()=>validatePidSettings({kp_x:-1}));
// IMU1 离线时，选中的 IMU2 仍可显示和控制；两路来源标签必须与地址一致。
const local=clone();local.control.source_id=2;
local.control.mount_yaw_deg=-45;
local.control_imu={...local.control_imu,address:0x69,source_id:2,source:'LOCAL_I2C_IMU2',g:[1,2,3]};
local.imus[0].present=local.imus[0].valid=false;
assert(isTelemetry(local),'absent unselected IMU1 cannot block IMU2');
vm.runInContext(`acceptPacket(${JSON.stringify(local)});`,context);
assert.equal(elements.get('use-imu2').checked,true);
assert.equal(elements.get('auto-start').disabled,false);
assert.equal(elements.get('primary-gyro').textContent,'1.00 / 2.00 / 3.00');
assert.equal(elements.get('active-source-name').textContent,'IMU2 · 0x69');
assert.match(elements.get('mounting-state').textContent,/IMU2 安装偏角 -45°.*已纳入姿态与 PID/);
bad=structuredClone(local);bad.control_imu.source='SUPERMINI_UART';assert(!isTelemetry(bad));
bad=structuredClone(local);bad.control_imu.valid=false;bad.world.valid=false;
vm.runInContext(`acceptPacket(${JSON.stringify(bad)});`,context);
assert.equal(elements.get('auto-start').disabled,true,'selected source failure never falls back');
// 实际网页的停止→切换→回传确认调用顺序，使用替代传输层，不连接硬件。
const commands=[];let device=structuredClone(local),rejectSwitch=false;
context.fakeSend=line=>{
  const request=decodeFrame(line.trimEnd());commands.push(request);
  const ok=!(rejectSwitch&&request.cmd==='select_imu');
  if(ok){device.epoch++;if(request.cmd==='select_imu'){
    device.control.source_id=request.source;device.control_imu={...packet.control_imu};
    device.calibrated=false;device.calibrating=true;device.world.valid=false;device.world.reference_id++;
  }}
  vm.runInContext(`acceptPacket(${JSON.stringify({v:VERSION,type:'ack',boot:device.boot,session:device.session,request_seq:request.seq,epoch:device.epoch,ok,reason:ok?'ACCEPTED':'IMU_NOT_FOUND'})});acceptPacket(${JSON.stringify(device)});`,context);
};
vm.runInContext("S.mode='wifi';S.ws={readyState:1,send:fakeSend};",context);
await vm.runInContext('switchImuSource(1)',context);
assert.deepEqual(commands.map(p=>p.cmd),['stop','select_imu']);
assert.equal(commands[1].source,1);
assert.equal(elements.get('use-imu2').checked,false);
assert.equal(elements.get('auto-start').disabled,true,'switch cannot restart outputs before calibration');
rejectSwitch=true;commands.length=0;
await assert.rejects(vm.runInContext('switchImuSource(2)',context),/目标 IMU 未检测到/);
assert.equal(elements.get('use-imu2').checked,false,'failed switch retains confirmed source');
// 改变模拟姿态、TVC 指向、轴交换和符号均不能发送任何设备命令。
const sentBeforePreview=commands.length,deviceBeforePreview=vm.runInContext('JSON.stringify(S.tele)',context);
elements.get('preview-mode').value='simulation';elements.get('preview-mode').events.change();
assert.match(elements.get('preview-status').textContent,/本地模拟/);
elements.get('sim-gimbal-mode').value='manual';elements.get('sim-gimbal-mode').events.change();
elements.get('sim-azimuth').value='90';elements.get('sim-elevation').value='150';elements.get('sim-elevation').events.input();
assert.match(elements.get('preview-elevation').textContent,/150.0/);
elements.get('map-axis-0').value='2';elements.get('map-axis-1').value='2';elements.get('map-axis-2').value='3';
elements.get('axis-preview-form').events.submit({preventDefault(){}});
assert.match(elements.get('axis-map-status').textContent,/不能重复/);
elements.get('map-axis-1').value='1';elements.get('axis-preview-form').events.submit({preventDefault(){}});
assert.match(elements.get('axis-map-status').textContent,/镜像/);
assert.equal(commands.length,sentBeforePreview);
assert.equal(vm.runInContext('JSON.stringify(S.tele)',context),deviceBeforePreview,'preview never rewrites actual attitude or PID telemetry');
elements.get('axis-reset').events.click();elements.get('preview-mode').value='live';elements.get('preview-mode').events.change();
// 原始故障不受临时通知覆盖，并能在断开后复制故障快照。
const faultPacket=clone();faultPacket.fault='PID_INVALID';faultPacket.state='FAULT';
vm.runInContext(`acceptPacket(${JSON.stringify(faultPacket)});notice('FAULT_REQUIRES_RESTART','error');`,context);
assert.equal(elements.get('device-fault-code').textContent,'PID_INVALID');
assert.match(elements.get('device-fault-description').textContent,/控制周期/);
assert.match(vm.runInContext("describeReason('FAULT_REQUIRES_RESTART')",context),/PID_INVALID/);
let copiedFault='';context.navigator={clipboard:{async writeText(text){copiedFault=text;}}};
await vm.runInContext('copyFault()',context);
assert.equal(JSON.parse(copiedFault).fault,'PID_INVALID');assert.equal(JSON.parse(copiedFault).session,undefined);
vm.runInContext('S.tele=null;S.mode=null;render();',context);
assert.equal(elements.get('device-fault-code').textContent,'PID_INVALID');assert.match(elements.get('device-fault-age').textContent,/历史故障快照/);
await vm.runInContext('copyFault()',context);assert.equal(JSON.parse(copiedFault).snapshot,true);
context.navigator={};await vm.runInContext('copyFault()',context);
assert.equal(elements.get('fault-copy-text').hidden,false);assert.match(elements.get('fault-copy-text').value,/PID_INVALID/);
assert.match(elements.get('fault-copy-status').textContent,/Ctrl\+C/);
vm.runInContext(`acceptPacket(${JSON.stringify(packet)});`,context);assert.equal(elements.get('device-fault-code').textContent,'无故障');
assert.equal(elements.get('copy-fault').disabled,true);
assert.equal(elements.get('fault-copy-text').hidden,true);
const warningPacket=clone();warningPacket.warning='HEARTBEAT_TIMEOUT';warningPacket.heartbeat_age_ms=1234;
warningPacket.operating_mode='MANUAL';warningPacket.state='SERVO_TEST';warningPacket.outputs.servo_enabled=true;
vm.runInContext(`S.mode='serial';S.heartbeat=true;acceptPacket(${JSON.stringify(warningPacket)});`,context);
assert.equal(elements.get('device-fault-code').textContent,'HEARTBEAT_TIMEOUT');assert.equal(elements.get('device-fault-code').className,'fault-warning');
assert.equal(vm.runInContext('S.heartbeat',context),true);
context.navigator={clipboard:{async writeText(text){copiedFault=text;}}};await vm.runInContext('copyFault()',context);
assert.equal(JSON.parse(copiedFault).fault,'');assert.equal(JSON.parse(copiedFault).warning,'HEARTBEAT_TIMEOUT');
vm.runInContext(`acceptPacket(${JSON.stringify(packet)});`,context);assert.equal(elements.get('device-fault-code').textContent,'无故障');
console.log('PASS dashboard protocol, channel mapping, DOM startup, stale data and IMU2 isolation checks. No serial hardware used.');
