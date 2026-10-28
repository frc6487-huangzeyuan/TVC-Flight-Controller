import * as preview from '../web/dist/preview.mjs';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {fileURLToPath} from 'node:url';
import {dirname,resolve} from 'node:path';
import vm from 'node:vm';
import * as protocol from '../web/dist/protocol.mjs';

const root=resolve(dirname(fileURLToPath(import.meta.url)),'..');
const good={address:0x69,present:true,valid:true,status:'OK',sample_t_us:999000,
  sample_hz:198.7,tilt_valid:true,temp_c:27.3,pitch_deg:12.4,roll_deg:-7.2};
let checks=0;
const check=(name,fn)=>{fn();checks++;console.log('PASS '+name);};
const display=m=>protocol.imu2Display(m,1000000,true);
check('IMU2 formats live measurements with units',()=>{
  assert.deepEqual(display(good),{status:'正常',live:true,pitch:'12.4°',roll:'-7.2°',temperature:'27.3 ℃',rate:'198.7 Hz'});
});
check('No connection and old firmware have explicit empty states',()=>{
  assert.equal(protocol.imu2Display(undefined,undefined,false).status,'等待连接');
  assert.equal(display(undefined).status,'未提供数据');
});
check('Absent sensor and missing interrupts cannot show fake zero angles',()=>{
  for(const [status,present,label] of [['NOT_FOUND',false,'未检测到'],['WAIT_INT',true,'等待中断'],['READ_ERROR',true,'读取异常'],['STALE',true,'数据过期']]){
    const view=display({...good,status,present,valid:false,tilt_valid:false});
    assert.equal(view.status,label);assert.equal(view.pitch,'—°');assert.equal(view.temperature,'— ℃');assert.equal(view.live,false);
  }
});
check('Both stale sensor and stale transport clear all readings',()=>{
  const stale=[display({...good,sample_t_us:950000}),protocol.imu2Display(good,1000000,false)];
  for(const view of stale){assert.equal(view.status,'数据过期');assert.equal(view.pitch,'—°');assert.equal(view.temperature,'— ℃');assert.equal(view.rate,'— Hz');}
});
check('10 Hz monitor remains live between 100 ms samples and expires at 300 ms',()=>{
  const lowRate={...good,sample_hz:10,target_hz:10,sample_timeout_us:300000};
  for(const age of [50000,99999,100000,199999,299999]){
    const view=display({...lowRate,sample_t_us:1000000-age});
    assert.equal(view.live,true);assert.equal(view.rate,'10.0 Hz');
  }
  for(const age of [300000,400000])
    assert.equal(display({...lowRate,sample_t_us:1000000-age}).status,'数据过期');
  assert.equal(protocol.imu2Display(lowRate,1000000,false).live,false);
  for(const sample_timeout_us of [0,49999,1000001,NaN,Infinity,'300000',null])
    assert.equal(display({...lowRate,sample_timeout_us}).status,'数据异常');
});
check('Invalid gravity tilt preserves only live temperature and rate',()=>{
  const view=display({...good,tilt_valid:false,pitch_deg:null,roll_deg:null});
  assert.equal(view.status,'倾角不可用');assert.equal(view.pitch,'—°');assert.equal(view.roll,'—°');assert.equal(view.temperature,'27.3 ℃');assert.equal(view.live,true);
});
check('Malformed optional monitor cannot render unsafe numbers',()=>{
  for(const m of [null,{},'bad',{...good,address:0x68},{...good,temp_c:NaN},{...good,sample_hz:-1},{...good,pitch_deg:Infinity},{...good,roll_deg:181},{...good,sample_t_us:1000001}]){
    const view=display(m);assert.equal(view.live,false);assert.equal(view.pitch,'—°');assert.equal(view.temperature,'— ℃');
  }
});

// 执行真实的网页渲染函数，遇到未知 DOM 元素编号即失败，且不连接硬件。
const html=readFileSync(resolve(root,'web/dist/index.html'),'utf8');
const elements=new Map([...html.matchAll(/id="([^"]+)"/g)].map(m=>[m[1],{
  value:m[1]==='motor-speed'?'25':'135',checked:true,textContent:'',disabled:false,
  getBoundingClientRect(){return {width:0};},classList:{toggle(){}},addEventListener(){}
}]));
let now=10000,sent=0;
const context={...protocol,...preview,TextEncoder,TextDecoder,console,performance:{now:()=>now},
  document:{getElementById(id){assert.ok(elements.has(id),'DOM id '+id);return elements.get(id);},hidden:false,addEventListener(){}},
  window:{addEventListener(){}},navigator:{},location:{hostname:'test.invalid'},
  WebSocket:{OPEN:1},setInterval(){},setTimeout,clearTimeout};
vm.createContext(context);
const app=readFileSync(resolve(root,'web/dist/app.mjs'),'utf8').replace(/^import[^\n]*\n/gm,'');
vm.runInContext(app+'\nglobalThis.testApi={S,render,acceptPacket};',context);
const {S,render,acceptPacket}=context.testApi;
const device=new protocol.DemoDevice(S.session);device.operatingMode='MANUAL';
const packet=device.telemetry();packet.t_us=1000000;packet.imus[0].sample_t_us=999000;
Object.assign(packet.imus[0],{source:'SUPERMINI_UART'});
packet.control_imu={...packet.imus[0],source_id:1,calibrated:false};
Object.assign(packet.control,{source_id:1,geometry:'AZIMUTH_ELEVATION',mount_ready:false,tilt_valid:false});
packet.outputs.servo_range_deg=[360,270];
S.mode='wifi';S.ws={readyState:1,send(){sent++;}};
check('Actual app renders IMU2 while original manual controls stay available',()=>{
  acceptPacket({...packet,imu2:good});
  assert.equal(elements.get('imu2-pitch').textContent,'12.4°');
  assert.equal(elements.get('imu2-status').textContent,'正常');
  assert.equal(elements.get('servo-enable').disabled,false);assert.equal(elements.get('motor-start').disabled,false);
});
check('IMU2 failure or malformed monitor never disables healthy IMU1 control',()=>{
  for(const imu2 of [undefined,null,{},'bad',{...good,present:false,valid:false,tilt_valid:false,status:'NOT_FOUND'}]){
    const p={...packet,imu2};assert.equal(protocol.isTelemetry(p),true);assert.equal(protocol.canOperate(p,0),true);
    acceptPacket(p);assert.equal(elements.get('servo-enable').disabled,false);assert.equal(elements.get('motor-start').disabled,false);
    assert.equal(elements.get('imu2-pitch').textContent,'—°');
  }
  assert.equal(sent,0,'Monitoring does not send actuator commands');
});
check('10 Hz IMU2 timeout does not relax the selected source declared validity',()=>{
  const imu2={...good,sample_hz:10,target_hz:10,sample_timeout_us:300000,sample_t_us:900000};
  acceptPacket({...packet,imu2});
  assert.equal(elements.get('imu2-status').textContent,'正常');
  assert.equal(elements.get('imu2-rate').textContent,'10.0 Hz');
  assert.equal(elements.get('servo-enable').disabled,false);
  const stalePrimary={...packet,imu2,control_imu:{...packet.control_imu,valid:false,sample_timeout_us:50000,sample_t_us:900000}};
  assert.equal(protocol.canOperate(stalePrimary,0),false);
  assert.equal(protocol.isTelemetry({...stalePrimary,control_imu:{...stalePrimary.control_imu,valid:true}}),false);
});
check('Actual app clears IMU2 when connection data expires or disconnects',()=>{
  acceptPacket({...packet,imu2:good});now+=3000;render();
  assert.equal(elements.get('imu2-status').textContent,'数据过期');assert.equal(elements.get('imu2-pitch').textContent,'—°');
  S.mode=null;S.tele=null;render();assert.equal(elements.get('imu2-status').textContent,'等待连接');
});
console.log(`${checks} IMU2 monitor checks passed. No hardware was connected.`);
