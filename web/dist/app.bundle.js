// 由 tools/embed_web.mjs 自动生成，请勿手动修改。
(function(){'use strict';
const {VERSION,encodeFrame,LineDecoder,isTelemetry,validateEscConfig,imu2Display,servoAngleToUs,servoUsToAngle,validatePidSettings,validateTiltThrottle,TRANSPORT_GRACE_MS}=(()=>{
const VERSION=10, MAX_FRAME=8192;
const TRANSPORT_GRACE_MS=2500;
const encoder=new TextEncoder();
function crc32(input){const bytes=typeof input==='string'?encoder.encode(input):input;let c=0xffffffff;for(const b of bytes){c^=b;for(let i=0;i<8;i++)c=(c>>>1)^((c&1)?0xedb88320:0);}return((c^0xffffffff)>>>0).toString(16).padStart(8,'0');}
function encodeFrame(packet){const body=JSON.stringify(packet);if(body.length>MAX_FRAME-10)throw new Error('FRAME_TOO_LONG');return `${body}\t${crc32(body)}\n`;}
function decodeFrame(line){if(line.length>MAX_FRAME)throw new Error('FRAME_TOO_LONG');const split=line.lastIndexOf('\t');if(split<0||!/^[0-9a-f]{8}$/i.test(line.slice(split+1)))throw new Error('FRAME_FORMAT');const body=line.slice(0,split);if(crc32(body)!==line.slice(split+1).toLowerCase())throw new Error('CRC');const p=JSON.parse(body);if(!p||p.v!==VERSION||!Number.isSafeInteger(p.seq)||p.seq<1)throw new Error('VERSION_OR_SEQUENCE');return p;}
class LineDecoder{
  constructor(){this.buffer='';this.decoder=new TextDecoder();this.discard=false;}
  push(bytes,onLine,onError){const chunk=typeof bytes==='string'?bytes:this.decoder.decode(bytes,{stream:true});for(const c of chunk){if(c==='\n'){if(this.discard)onError(new Error('FRAME_TOO_LONG'));else if(this.buffer.trim()&&!this.buffer.startsWith('#IMU1 ')){try{onLine(decodeFrame(this.buffer.replace(/\r$/,'')));}catch(e){onError(e);}}this.buffer='';this.discard=false;}else if(!this.discard){this.buffer+=c;if(this.buffer.length>MAX_FRAME){this.buffer='';this.discard=true;}}}}
}
const vector=v=>Array.isArray(v)&&v.length===3&&v.every(Number.isFinite);
function servoAngleToUs(angle,channel){const range=channel===0?360:270;const n=Number(angle),low=channel===1?85:0,high=channel===1?185:360;if(!Number.isFinite(n)||n<low||n>high)throw new Error(channel===1?'俯仰限位：中位 ±50°（85～185°）':'舵机角度超出范围');return Math.round(500+n*2000/range);}
function servoUsToAngle(us,channel){return (Number(us)-500)*(channel===0?360:270)/2000;}
function validatePidSettings(c){
  for(const key of ['kp_x','ki_x','kd_x','kp_y','ki_y','kd_y'])if(!Number.isFinite(c[key])||c[key]<0||c[key]>(key.startsWith('kp')?10:2))throw new Error('PID 参数超出试调范围');
  if(!Number.isFinite(c.max_tilt_deg)||c.max_tilt_deg<1||c.max_tilt_deg>50)throw new Error('最大俯仰偏转须为1–50°');
  if(!Number.isFinite(c.azimuth_zero_deg)||c.azimuth_zero_deg<0||c.azimuth_zero_deg>360||![1,-1].includes(c.sign_x)||![1,-1].includes(c.sign_y))throw new Error('云台零位或方向设置不正确');
  return c;
}
function validateTiltThrottle(c){
  if(!['base_pct','gain_pct_deg','max_pct','deadband_deg','ramp_pct_s'].every(k=>Number.isFinite(c[k]))||c.base_pct<0||c.base_pct>c.max_pct||c.max_pct<=0||c.max_pct>100||c.gain_pct_deg<0||c.gain_pct_deg>5||c.deadband_deg<0||c.deadband_deg>30||c.ramp_pct_s<1||c.ramp_pct_s>100)throw new Error('自动油门参数无效：基础值不能超过上限，增益 0～5%/°，死区 0～30°，变化速度 1～100%/秒');
  return c;
}
// 独立验证可选的监测数据，IMU2 不参与 IMU1 控制的启用判断。
function imu2Display(m,nowUs,transportFresh){
  const empty=status=>({status,live:false,pitch:'—°',roll:'—°',temperature:'— ℃',rate:'— Hz'});
  if(!transportFresh)return empty(nowUs===undefined?'等待连接':'数据过期');
  if(m===undefined)return empty('未提供数据');
  if(!m||m.address!==0x69||typeof m.present!=='boolean'||typeof m.valid!=='boolean'||typeof m.tilt_valid!=='boolean'||!Number.isSafeInteger(m.sample_t_us)||m.sample_t_us<0||!Number.isSafeInteger(nowUs)||m.sample_t_us>nowUs)return empty('数据异常');
  if(!m.present)return empty(m.status==='NOT_FOUND'&&!m.valid&&!m.tilt_valid?'未检测到':'数据异常');
  if(!m.valid){const statuses={WAIT_INT:'等待中断',READ_ERROR:'读取异常',STALE:'数据过期'};return empty(!m.tilt_valid&&Object.hasOwn(statuses,m.status)?statuses[m.status]:'数据异常');}
  // 旧固件的监测超时为 50 ms；10 Hz 监测固件会明确提供
  // 其独立的 300 ms 超时值；IMU1 的历史窗口由主控单独报告。
  const timeoutUs=m.sample_timeout_us===undefined?50000:m.sample_timeout_us;
  if(!Number.isSafeInteger(timeoutUs)||timeoutUs<50000||timeoutUs>1000000)return empty('数据异常');
  if(m.status!=='OK'||nowUs-m.sample_t_us>=timeoutUs)return empty('数据过期');
  if(!Number.isFinite(m.temp_c)||!Number.isFinite(m.sample_hz)||m.sample_hz<0)return empty('数据异常');
  if(m.tilt_valid&&(!Number.isFinite(m.pitch_deg)||Math.abs(m.pitch_deg)>90||!Number.isFinite(m.roll_deg)||Math.abs(m.roll_deg)>180))return empty('数据异常');
  return {status:m.tilt_valid?'正常':'倾角不可用',live:true,
    pitch:m.tilt_valid?`${m.pitch_deg.toFixed(1)}°`:'—°',roll:m.tilt_valid?`${m.roll_deg.toFixed(1)}°`:'—°',
    temperature:`${m.temp_c.toFixed(1)} ℃`,rate:`${m.sample_hz.toFixed(1)} Hz`};
}
function isTelemetry(p){
  if(p?.warning!==undefined&&typeof p.warning!=='string')return false;
  if(!p||p.v!==VERSION||p.imu_count!==1||p.type!=='telemetry'||typeof p.boot!=='string'||typeof p.session!=='string'||!Number.isSafeInteger(p.t_us)||p.t_us<0||!['DISARMED','SERVO_TEST','ARMING','ARMED','FAULT'].includes(p.state)||!['ATTITUDE','MANUAL'].includes(p.operating_mode)||typeof p.fault!=='string'||!Number.isInteger(p.epoch)||!Number.isInteger(p.config_rev)||typeof p.firmware!=='string'||typeof p.calibrating!=='boolean'||typeof p.calibrated!=='boolean')return false;
  const sensorValid=(s,address,source)=>{
    const timeout=s?.sample_timeout_us??50000;
    return s&&s.address===address&&s.source===source&&typeof s.valid==='boolean'&&typeof s.present==='boolean'&&vector(s.a)&&vector(s.g)&&Number.isFinite(s.pitch)&&Number.isFinite(s.roll)&&Number.isSafeInteger(s.sample_t_us)&&s.sample_t_us>=0&&s.sample_t_us<=p.t_us&&Number.isSafeInteger(timeout)&&timeout>=50000&&timeout<=2000000&&(!s.valid||(s.present&&p.t_us-s.sample_t_us<timeout));
  };
  if(!Array.isArray(p.imus)||p.imus.length!==1||!sensorValid(p.imus[0],0x68,'SUPERMINI_UART'))return false;
  const source=p.control?.source_id;
  if(![1,2].includes(source)||p.control_imu?.source_id!==source||!sensorValid(p.control_imu,source===1?0x68:0x69,source===1?'SUPERMINI_UART':'LOCAL_I2C_IMU2'))return false;
  const w=p.world;
  if(!w||w.frame!=='LOCAL_Z_UP'||typeof w.valid!=='boolean'||w.heading_absolute!==false||!Number.isSafeInteger(w.reference_id)||w.reference_id<0||typeof w.correction_active!=='boolean'||!Number.isFinite(w.reference_age_s)||w.reference_age_s<0)return false;
  if(typeof w.heading_defined!=='boolean'||![w.euler_deg,w.a_body_g,w.gyro_body_dps].every(vector))return false;
  const worldVector=v=>vector(v)||(!w.valid&&Array.isArray(v)&&v.length===3&&v.every(n=>n===null||Number.isFinite(n)));
  if(!worldVector(w.f_world_g)||!worldVector(w.linear_world_mps2))return false;
  if(!Array.isArray(w.q_bw)||w.q_bw.length!==4||!w.q_bw.every(Number.isFinite)||Math.abs(Math.hypot(...w.q_bw)-1)>.01)return false;
  if(!Array.isArray(w.r_bw)||w.r_bw.length!==9||!w.r_bw.every(Number.isFinite))return false;
  if(w.valid&&(!p.calibrated||!p.control_imu.valid||!p.control_imu.calibrated||w.reference_id<1))return false;
  if(!p.control||typeof p.control.auto_motor_allowed!=='boolean'||p.control.geometry!=='AZIMUTH_ELEVATION'||typeof p.control.mount_ready!=='boolean'||typeof p.control.tilt_valid!=='boolean')return false;
  const o=p.outputs;
  if(!o||typeof o.servo_enabled!=='boolean'||!Array.isArray(o.servo_us)||o.servo_us.length!==2||!o.servo_us.every(Number.isFinite)||typeof o.esc_armed!=='boolean'||typeof o.esc_arming!=='boolean'||typeof o.esc_configured!=='boolean'||typeof o.esc_signal_enabled!=='boolean'||!Number.isFinite(o.esc_us))return false;
  if(o.servo_range_deg?.[0]!==360||o.servo_range_deg?.[1]!==270||p.imus[0].source!=='SUPERMINI_UART')return false;
  if(o.esc_configured&&(!Number.isInteger(o.esc_stop_us)||o.esc_stop_us<900||o.esc_stop_us>2100||!Number.isInteger(o.esc_min_us)||o.esc_min_us<900||o.esc_min_us>=o.esc_stop_us||!Number.isInteger(o.esc_max_us)||o.esc_max_us<=o.esc_stop_us||o.esc_max_us>2100))return false;
  if((o.esc_armed||o.esc_arming)&&(!o.esc_configured||!o.esc_signal_enabled))return false;
  const tt=p.control.tilt_throttle;
  if(tt!==undefined){
    if(!tt||typeof tt.enabled!=='boolean'||!['tilt_deg','target_pct','output_pct'].every(k=>Number.isFinite(tt[k]))||tt.tilt_deg<0||tt.tilt_deg>180||tt.target_pct<0||tt.output_pct<0||tt.target_pct>tt.max_pct+.001||tt.output_pct>tt.max_pct+.001)return false;
    try{validateTiltThrottle(tt);}catch{return false;}
    if(tt.enabled&&(!p.control.auto_enabled||!o.servo_enabled||!o.esc_armed||o.esc_arming||p.operating_mode!=='ATTITUDE'||p.fault||!w.valid))return false;
  }
  return true;
}
function validateEscConfig(c){for(const key of ['stop_us','min_us','max_us'])if(!Number.isInteger(c[key])||c[key]<900||c[key]>2100)throw new Error('脉宽必须是900–2100之间的整数');if(c.min_us>=c.stop_us||c.max_us<=c.stop_us)throw new Error('中位停止值必须位于反向端与正向端之间');if(!Number.isInteger(c.hold_ms)||c.hold_ms<1000||c.hold_ms>15000||c.hz!==50)throw new Error('保持时间须为1000–15000ms，仅支持50Hz PWM');if(c.confirmed!==true)throw new Error('请先确认已拆桨，并核对说明书');return c;}
function canOperate(packet,ageMs){return !!(isTelemetry(packet)&&packet.operating_mode==='MANUAL'&&ageMs<TRANSPORT_GRACE_MS&&!packet.fault&&!packet.calibrating&&packet.control_imu.valid);}
function controlLeaseValid(start,current){return start.generation===current.generation&&start.session===current.session&&current.heartbeatEnabled===true&&current.hidden===false;}
class DemoDevice{
  constructor(session){this.session=session;this.boot='simulation-only';this.seq=0;this.epoch=0;this.rev=0;this.start=performance.now();this.fault='';this.operatingMode='ATTITUDE';this.servo=false;this.auto=false;this.autoMotorAllowed=false;this.x=1500;this.y=1500;this.config=null;this.armed=false;this.armingAt=0;this.us=0;this.signal=false;this.calibrateUntil=0;this.worldReady=false;this.referenceId=0;this.referenceAt=performance.now();this.yawZero=0;this.source=2;this.tiltThrottle={enabled:false,base_pct:0,gain_pct_deg:1,max_pct:30,deadband_deg:2,ramp_pct_s:10,tilt_deg:0,target_pct:0,output_pct:0};}
  stopServo(){if(this.tiltThrottle.enabled)this.stopEsc();this.servo=false;this.auto=false;this.epoch++;}
  stopEsc(){this.tiltThrottle.enabled=false;this.tiltThrottle.output_pct=0;this.armed=false;this.armingAt=0;this.epoch++;this.us=this.signal?(this.config?.stop_us??0):0;}
  stop(){this.autoMotorAllowed=false;this.stopServo();this.stopEsc();}
  motorAllowed(){return this.operatingMode==='MANUAL'||(this.operatingMode==='ATTITUDE'&&this.autoMotorAllowed&&this.worldReady);}
  command(p){let ok=true,reason='ACCEPTED';
    if(!['hello','heartbeat','keepalive','stop','servo_disable','esc_stop','auto_disable','auto_motor_disable','tilt_throttle_disable'].includes(p.cmd)&&p.epoch!==this.epoch){ok=false;reason='OLD_EPOCH';}
    else if(p.cmd==='hello'){this.stop();reason='HELLO';}else if(p.cmd==='keepalive')reason='KEEPALIVE';else if(p.cmd==='heartbeat')reason='HEARTBEAT';else if(p.cmd==='stop'){this.stop();reason='STOPPED';}else if(p.cmd==='clear_fault'){this.fault='';this.stop();}
    else if(p.cmd==='set_mode'){if(p.mode==='ATTITUDE'){this.stop();this.operatingMode='ATTITUDE';this.calibrateUntil=performance.now()+3000;this.worldReady=false;reason='MODE_ATTITUDE_CALIBRATING';}else if(p.mode==='MANUAL'){this.stop();this.operatingMode='MANUAL';this.calibrateUntil=0;if(['CALIBRATION_MOVED','CALIBRATION_DATA','HEADING_UNDEFINED'].includes(this.fault))this.fault='';reason='MODE_MANUAL';}else{ok=false;reason='INVALID_MODE';}}
    else if(p.cmd==='calibrate'){if(this.servo||this.armed||this.armingAt){ok=false;reason='STOP_OUTPUTS_FIRST';}else{this.autoMotorAllowed=false;this.epoch++;this.calibrateUntil=performance.now()+3000;this.worldReady=false;}}
    else if(p.cmd==='world_zero'){if(!this.worldReady||this.servo||this.armed||this.armingAt){ok=false;reason='WORLD_NOT_READY';}else{this.yawZero=15*Math.sin((performance.now()-this.start)/1000*.12);this.referenceId++;this.referenceAt=performance.now();this.epoch++;}}
    else if(p.cmd==='servo_enable'){if(this.operatingMode!=='MANUAL'){ok=false;reason='MODE_LOCKED';}else if(this.calibrateUntil||this.fault||this.servo){ok=false;reason='NOT_READY';}else{this.servo=true;this.auto=false;this.epoch++;this.x=this.y=1500;}}
    else if(p.cmd==='servo_disable'){if(this.operatingMode!=='MANUAL'){ok=false;reason='MODE_LOCKED';}else this.stopServo();}
    else if(p.cmd==='servo'){if(this.operatingMode!=='MANUAL'){ok=false;reason='MODE_LOCKED';}else if(!this.servo||p.epoch!==this.epoch||!Number.isInteger(p.x)||!Number.isInteger(p.y)||p.x<500||p.x>2500||p.y<500||p.y>2500){ok=false;reason='SERVO_NOT_READY';}else{this.x=p.x;this.y=p.y;}}
    else if(p.cmd==='esc_config'){try{if(!this.motorAllowed())throw new Error('MODE_LOCKED');validateEscConfig(p);if(this.armed||this.armingAt)throw new Error('STOP_ESC_FIRST');this.stopEsc();this.config={...p};this.rev++;this.signal=false;this.us=0;}catch(e){ok=false;reason=e.message;}}
    else if(p.cmd==='esc_arm'){if(!this.motorAllowed()){ok=false;reason='MODE_LOCKED';}else if(this.calibrateUntil||this.fault||!this.config||p.config_rev!==this.rev){ok=false;reason='NOT_CONFIGURED';}else{this.stopEsc();this.signal=true;this.us=this.config.stop_us;this.armingAt=performance.now()+this.config.hold_ms;}}
    else if(p.cmd==='esc'){if(this.tiltThrottle.enabled){ok=false;reason='AUTO_THROTTLE_ACTIVE';}else if(!this.motorAllowed()){ok=false;reason='MODE_LOCKED';}else if(!this.armed||p.epoch!==this.epoch||!Number.isInteger(p.us)||p.us<this.config.min_us||p.us>this.config.max_us){ok=false;reason='NOT_ARMED_OR_RANGE';}else this.us=p.us;}
    else if(p.cmd==='esc_stop'){this.stopEsc();}
    else if(p.cmd==='tilt_throttle_disable'){this.stopEsc();this.autoMotorAllowed=false;}
    else if(p.cmd==='tilt_throttle_config'){try{if(this.servo||this.armed||this.armingAt)throw Error('STOP_OUTPUTS_FIRST');validateTiltThrottle(p);for(const key of ['base_pct','gain_pct_deg','max_pct','deadband_deg','ramp_pct_s'])this.tiltThrottle[key]=p[key];this.tiltThrottle.enabled=false;this.epoch++;}catch(e){ok=false;reason=e.message;}}

    else if(p.cmd==='auto_motor_enable'){if(this.operatingMode!=='ATTITUDE'){ok=false;reason='MODE_LOCKED';}else if(!this.worldReady||this.fault||this.calibrateUntil){ok=false;reason='NOT_READY';}else{this.autoMotorAllowed=true;this.epoch++;}}
    else if(p.cmd==='auto_motor_disable'){if(this.operatingMode!=='ATTITUDE'){ok=false;reason='MODE_LOCKED';}else{this.autoMotorAllowed=false;this.stopEsc();}}
    else if(p.cmd==='auto_enable'){if(this.operatingMode!=='ATTITUDE'){ok=false;reason='MODE_LOCKED';}else if(!this.worldReady||this.fault||((this.armed||this.armingAt)&&!this.autoMotorAllowed)){ok=false;reason='NOT_READY';}else if(p.tilt_throttle&&(!this.armed||!this.autoMotorAllowed)){ok=false;reason='THROTTLE_NOT_READY';}else{this.tiltThrottle.enabled=!!p.tilt_throttle;this.servo=true;this.auto=true;this.epoch++;this.x=this.y=1500;}}
    else if(p.cmd==='auto_disable'){if(this.operatingMode!=='ATTITUDE'){ok=false;reason='MODE_LOCKED';}else this.stop();}
    else if(p.cmd==='select_imu'){if(![1,2].includes(p.source)){ok=false;reason='INVALID_IMU_SOURCE';}else if(p.source===this.source){reason='IMU_SOURCE_UNCHANGED';}else{this.stop();this.source=p.source;this.worldReady=false;this.calibrateUntil=performance.now()+3000;this.epoch++;reason='IMU_SOURCE_SELECTED';}}
    else{ok=false;reason='UNKNOWN_COMMAND';}
    return {v:VERSION,type:'ack',seq:++this.seq,request_seq:p.seq,session:this.session,boot:this.boot,ok,reason,epoch:this.epoch,config_rev:this.rev,firmware:'SIMULATION / 模拟'};
  }
  telemetry(){const now=performance.now();if(this.armingAt&&now>=this.armingAt){this.armingAt=0;this.armed=true;}const t=(now-this.start)/1000;
    if(this.calibrateUntil&&now>=this.calibrateUntil){this.calibrateUntil=0;this.worldReady=true;this.referenceId++;this.referenceAt=now;this.yawZero=15*Math.sin(t*.12);}
    const pitch=8*Math.sin(t*.75),roll=5*Math.sin(t*.5+.6),yaw=15*Math.sin(t*.12)-this.yawZero;
    const world=demoWorld(roll,pitch,yaw,this.worldReady,this.referenceId,(now-this.referenceAt)/1000);
    const sample=address=>({address,source:address===0x69?'LOCAL_I2C_IMU2':'SUPERMINI_UART',source_id:address===0x69?2:1,
      present:true,valid:true,calibrated:this.worldReady,sample_t_us:Math.max(0,Math.round(t*1e6)-650),
      samples:Math.round(t*200),errors:0,missed:0,read_errors:0,race_drops:0,pitch,roll,temperature:27,
      a:[-Math.sin(pitch*Math.PI/180),Math.sin(roll*Math.PI/180),Math.cos(pitch*Math.PI/180)*Math.cos(roll*Math.PI/180)],
      g:[2.5*Math.cos(t*.5+.6),6*Math.cos(t*.75),0],bias:[0,0,0],
      sample_timeout_us:2000000,fresh_window_us:50000,history_state:'LIVE',status:'OK',sample_hz:200,target_hz:200,
      uart_replies:address===0x68?Math.round(t*200):undefined,uart_bad_frames:0,uart_timeouts:0});
    const imu1=sample(0x68),controlImu=sample(this.source===1?0x68:0x69);
    controlImu.control_source=true;imu1.control_source=this.source===1;
    return {v:VERSION,type:'telemetry',seq:++this.seq,session:this.session,boot:this.boot,t_us:Math.round(t*1e6),firmware:'SIMULATION / 模拟',fault:this.fault,epoch:this.epoch,config_rev:this.rev,state:this.armingAt?'ARMING':this.armed?'ARMED':this.servo?'SERVO_TEST':'DISARMED',operating_mode:this.operatingMode,calibrating:now<this.calibrateUntil,calibrated:this.worldReady,imu_count:1,
      control_source:this.source,control_source_name:'IMU'+this.source,
      world,control:{auto_enabled:this.auto,auto_motor_allowed:this.autoMotorAllowed,error_deg:[roll,pitch],zero_deg:[0,0],source_id:this.source,source_address:this.source===1?0x68:0x69,tilt_throttle:{...this.tiltThrottle},geometry:'AZIMUTH_ELEVATION',mount_ready:this.worldReady,tilt_valid:this.worldReady},
      sample_hz:200,bad_frames:0,
      outputs:{servo_enabled:this.servo,servo_us:[this.x,this.y],servo_range_deg:[360,270],esc_configured:!!this.config,esc_signal_enabled:this.signal,esc_armed:this.armed,esc_arming:!!this.armingAt,esc_us:this.us,esc_stop_us:this.config?.stop_us??0,esc_min_us:this.config?.min_us??0,esc_max_us:this.config?.max_us??0},
      imus:[imu1],control_imu:controlImu,
      imu2:{address:0x69,present:true,valid:true,tilt_valid:true,sample_t_us:Math.max(0,Math.round(t*1e6)-500),sample_hz:200,target_hz:200,sample_timeout_us:300000,status:'OK',temp_c:27.4,pitch_deg:pitch,roll_deg:roll,a_g:[0,0,1],g_dps:[0,0,0],control_source:this.source===2}};

  }
}
// 四元数和矩阵约定与固件一致：机体坐标转换到局部世界坐标，Z 轴向上。
function demoWorld(roll,pitch,yaw,valid=true,referenceId=1,referenceAge=0){
  const [r,p,y]=[roll,pitch,yaw].map(a=>a*Math.PI/360),cr=Math.cos(r),sr=Math.sin(r),cp=Math.cos(p),sp=Math.sin(p),cy=Math.cos(y),sy=Math.sin(y);
  const q=[cy*cp*cr+sy*sp*sr,cy*cp*sr-sy*sp*cr,cy*sp*cr+sy*cp*sr,sy*cp*cr-cy*sp*sr];
  const [w,x,zY,z]=q;
  const R=[1-2*(zY*zY+z*z),2*(x*zY-w*z),2*(x*z+w*zY),2*(x*zY+w*z),1-2*(x*x+z*z),2*(zY*z-w*x),2*(x*z-w*zY),2*(zY*z+w*x),1-2*(x*x+zY*zY)];
  return {valid,frame:'LOCAL_Z_UP',reference_id:referenceId,heading_absolute:false,heading_defined:Math.abs(Math.cos(pitch*Math.PI/180))>=.01,correction_active:true,reference_age_s:referenceAge,q_bw:q,r_bw:R,euler_deg:[roll,pitch,yaw],a_body_g:[R[6],R[7],R[8]],gyro_body_dps:[0,0,0],f_world_g:valid?[0,0,1]:[null,null,null],linear_world_mps2:valid?[0,0,0]:[null,null,null]};
}
function isFreshCommandTime(sentUs,nowUs){return Number.isSafeInteger(sentUs)&&sentUs>=0&&sentUs<=nowUs+100000&&nowUs<=sentUs+500000;}
function telemetryToCsv(records){
  const headers=['mode','received_iso','firmware','boot_id','t_us','operating_mode','state','fault','servo_enabled','servo_x_us','servo_y_us','esc_armed','esc_us'];
  for(const k of ['valid','sample_t_us','pitch_deg','roll_deg','ax_g','ay_g','az_g','gx_dps','gy_dps','gz_dps','errors','missed'])headers.push('imu1_'+k);
  headers.push('world_valid','world_reference_id','world_reference_age_s','heading_absolute','gravity_correction','world_roll_deg','world_pitch_deg','world_yaw_deg','qw','qx','qy','qz','body_ax_g','body_ay_g','body_az_g','body_gx_dps','body_gy_dps','body_gz_dps','world_fx_g','world_fy_g','world_fz_g','world_ax_mps2','world_ay_mps2','world_az_mps2');
  const quote=v=>'"'+String(v??'').replaceAll('"','""')+'"';
  const rows=records.map(p=>{const o=p.outputs,s=p.imus[0],w=p.world;return [p.mode,p.received,p.firmware,p.boot,p.t_us,p.operating_mode,p.state,p.fault,o.servo_enabled,...o.servo_us,o.esc_armed,o.esc_us,s.valid,s.sample_t_us,s.pitch,s.roll,...s.a,...s.g,s.errors,s.missed,w.valid,w.reference_id,w.reference_age_s,w.heading_absolute,w.correction_active,...w.euler_deg,...w.q_bw,...w.a_body_g,...w.gyro_body_dps,...w.f_world_g,...w.linear_world_mps2].map(quote).join(',');});
  return '\ufeff'+[headers.join(','),...rows].join('\r\n');
}

return {VERSION,encodeFrame,LineDecoder,isTelemetry,validateEscConfig,imu2Display,servoAngleToUs,servoUsToAngle,validatePidSettings,validateTiltThrottle,TRANSPORT_GRACE_MS};})();
const {createAttitudePreview}=(()=>{
// 纯几何预览：不包含串口、WebSocket、硬件命令或真实飞行动力学。
// R 使用行优先数组，v_world = R * v_body；机体 +Z 指向箭头。
const RAD=Math.PI/180,IDENTITY=[1,0,0,0,1,0,0,0,1];
const add=(a,b)=>a.map((v,i)=>v+b[i]),scale=(a,k)=>a.map(v=>v*k);
const clamp=(v,a,b)=>Math.max(a,Math.min(b,v));
function transpose3(m){return [m[0],m[3],m[6],m[1],m[4],m[7],m[2],m[5],m[8]];}
function multiply3(a,b){return Array.from({length:9},(_,i)=>{const r=Math.floor(i/3),c=i%3;return a[r*3]*b[c]+a[r*3+1]*b[c+3]+a[r*3+2]*b[c+6];});}
function transform3(m,v){return [0,1,2].map(r=>m[3*r]*v[0]+m[3*r+1]*v[1]+m[3*r+2]*v[2]);}
function determinant3(m){return m[0]*(m[4]*m[8]-m[5]*m[7])-m[1]*(m[3]*m[8]-m[5]*m[6])+m[2]*(m[3]*m[7]-m[4]*m[6]);}
function axisMapping(axes){
  if(!Array.isArray(axes)||axes.length!==3||axes.some(n=>!Number.isInteger(n)||Math.abs(n)<1||Math.abs(n)>3)||new Set(axes.map(Math.abs)).size!==3)throw new Error('X、Y、Z 必须各使用一次，不能重复选轴。');
  const m=Array(9).fill(0);axes.forEach((n,r)=>m[r*3+Math.abs(n)-1]=Math.sign(n));return m;
}
function eulerMatrix(roll,pitch,yaw){
  const [r,p,y]=[roll,pitch,yaw].map(v=>v*RAD),cr=Math.cos(r),sr=Math.sin(r),cp=Math.cos(p),sp=Math.sin(p),cy=Math.cos(y),sy=Math.sin(y);
  return [cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr,sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr,-sp,cp*sr,cp*cr];
}
// 从原机体姿态先恢复传感器的世界朝向，再改变“传感器→预览机体”的对应关系。
function remapAttitude(bodyToWorld,sensorToBody,previewSensorToBody){return multiply3(multiply3(bodyToWorld,sensorToBody),transpose3(previewSensorToBody));}
function gimbalDirection(azimuth,elevation,zero=180){
  const a=(azimuth-zero)*RAD,t=(elevation-135)*RAD;
  return [Math.sin(t)*Math.cos(a),Math.sin(t)*Math.sin(a),Math.cos(t)];
}
// 与 GimbalControl.h 的静态 P 分量和几何选解一致；不冒充完整动态 PID 仿真。
function staticGimbal(r,settings,current=180){
  const lean=[Math.atan2(-r[6],r[8])/RAD,Math.atan2(-r[7],r[8])/RAD];
  const vx=-lean[0]*settings.kp_x*settings.sign_x,vy=-lean[1]*settings.kp_y*settings.sign_y;
  const mag=Math.hypot(vx,vy),amount=Math.min(mag,settings.max_tilt_deg);
  if(mag<.15)return {azimuth:current,elevation:135,lean};
  const direction=Math.atan2(vy,vx)/RAD+settings.azimuth_zero_deg;
  let best=Infinity,azimuth=current,elevation=135;
  for(let flip=0;flip<2;flip++)for(let turn=-2;turn<=2;turn++){
    const candidate=direction+180*flip+360*turn,distance=Math.abs(candidate-current);
    if(candidate>=0&&candidate<=360&&distance<best){best=distance;azimuth=candidate;elevation=135+(flip?-amount:amount);}
  }
  return {azimuth,elevation,lean};
}
const validMatrix=m=>Array.isArray(m)&&m.length===9&&m.every(Number.isFinite);
const fmt=(n,d=1)=>Number.isFinite(n)?n.toFixed(d):'—';
const vector=v=>v.map(n=>fmt(n,3)).join(' / ');
const COLORS=['#ff8c87','#55ddbc','#7caeff'];

// SVG 三维投影，无 CDN、WebGL 或外部模型文件；适用于固件热点和离线单文件。
function sceneSvg(r,azimuth,elevation,{camera=42,view='orbit',sensorMatrix=IDENTITY,zero=180,valid=true,mirrored=false}={}){
  const yaw=camera*RAD,pitch=(view==='top'?85:view==='side'?4:23)*RAD;
  const right=[Math.cos(yaw),Math.sin(yaw),0],depth=[-Math.sin(yaw)*Math.cos(pitch),Math.cos(yaw)*Math.cos(pitch),-Math.sin(pitch)],up=[-Math.sin(yaw)*Math.sin(pitch),Math.cos(yaw)*Math.sin(pitch),Math.cos(pitch)];
  const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0);
  const project=p=>[430+dot(p,right)*135,282-dot(p,up)*135,dot(p,depth)];
  const points=ps=>ps.map(p=>project(p).slice(0,2).map(n=>n.toFixed(2)).join(',')).join(' ');
  const segment=(a,b,color,width=1,dash='')=>`<polyline points="${points([a,b])}" fill="none" stroke="${color}" stroke-width="${width}" ${dash?'stroke-dasharray="'+dash+'"':''}/>`;
  const text=(p,label,color,size=13)=>{const [x,y]=project(p);return `<text x="${x.toFixed(2)}" y="${y.toFixed(2)}" fill="${color}" font-size="${size}" font-family="system-ui,Microsoft YaHei" stroke="#0a1422" stroke-width="3" paint-order="stroke">${label}</text>`;};
  const arrow=(a,b,color,width=3)=>{const [x,y]=project(a),[u,v]=project(b),t=Math.atan2(v-y,u-x),d=10;
    return segment(a,b,color,width)+`<path d="M${u},${v} L${u-d*Math.cos(t-.4)},${v-d*Math.sin(t-.4)} L${u-d*Math.cos(t+.4)},${v-d*Math.sin(t+.4)} Z" fill="${color}"/>`;};
  let grid='';for(let i=-4;i<=4;i++){const n=i*.45;grid+=segment([-1.8,n,-1.2],[1.8,n,-1.2],'#24374a')+segment([n,-1.8,-1.2],[n,1.8,-1.2],'#24374a');}
  const origin=[-1.85,-.8,-1.2];let world='';
  for(let k=0;k<3;k++){const v=[0,0,0];v[k]=.52;world+=arrow(origin,add(origin,v),COLORS[k],2)+text(add(add(origin,v),[.04,0,.04]),`W+${'XYZ'[k]}`,COLORS[k],11);}
  const faces=[],body=p=>transform3(r,p);
  function face(ps,fill,opacity=1){const worldPs=ps.map(body);faces.push({z:worldPs.reduce((sum,p)=>sum+project(p)[2],0)/ps.length,markup:`<polygon points="${points(worldPs)}" fill="${fill}" fill-opacity="${opacity}" stroke="#26394e" stroke-width=".6"/>`});}
  const ring=(i,z,radius)=>[radius*Math.cos(i*Math.PI/8),radius*Math.sin(i*Math.PI/8),z];
  for(let i=0;i<16;i++){
    const shade=Math.round(145+38*Math.cos(i*Math.PI/8+.4));
    face([ring(i,-.85,.19),ring(i+1,-.85,.19),ring(i+1,1.05,.19),ring(i,1.05,.19)],`rgb(${shade},${shade+16},${shade+29})`);
    face([ring(i,1.05,.19),ring(i+1,1.05,.19),[0,0,1.58]],'#4cd2c1');
    face([ring(i,.32,.196),ring(i+1,.32,.196),ring(i+1,.51,.196),ring(i,.51,.196)],'#1d6a76');
    face([ring(i,-.85,.19),ring(i+1,-.85,.19),ring(i+1,-.96,.16),ring(i,-.96,.16)],'#293e52');
  }
  for(let i=0;i<4;i++){
    const t=i*Math.PI/2,cs=Math.cos(t),sn=Math.sin(t),fin=(radius,z)=>[radius*cs,radius*sn,z];
    face([fin(.18,-.22),fin(.5,-.85),fin(.18,-.85)],'#3e687f');
  }
  const dir=gimbalDirection(azimuth,elevation,zero),pivot=[0,0,-.99],rim=add(pivot,scale(dir,-.32));
  const normal=Math.hypot(dir[0],dir[1])>.01?[-dir[1],dir[0],0]:[1,0,0],norm=Math.hypot(...normal),u=scale(normal,1/norm),v=[dir[1]*u[2]-dir[2]*u[1],dir[2]*u[0]-dir[0]*u[2],dir[0]*u[1]-dir[1]*u[0]];
  const nozzle=(i,c,rad)=>add(c,add(scale(u,rad*Math.cos(i*Math.PI/6)),scale(v,rad*Math.sin(i*Math.PI/6))));
  for(let i=0;i<12;i++)face([nozzle(i,pivot,.08),nozzle(i+1,pivot,.08),nozzle(i+1,rim,.16),nozzle(i,rim,.16)],i%2?'#ba873d':'#e7b768');
  faces.sort((a,b)=>b.z-a.z);
  const showAxes=(transform,base,length,prefix)=>{
    let bits='';
    for(let k=0;k<3;k++){
      const axis=[0,0,0];axis[k]=prefix==='B'&&k===2?1.85:length;const end=add(base,transform3(transform,axis)),neg=add(base,scale(transform3(transform,axis),-.65));
      bits+=arrow(base,end,COLORS[k],prefix==='S'?1.8:2.5)+segment(base,neg,COLORS[k],1.5,'5 5')+text(end,`${prefix}+${'XYZ'[k]}`,COLORS[k],prefix==='S'?13:15)+text(neg,`${prefix}−${'XYZ'[k]}`,COLORS[k],12);
    }
    return bits;
  };
  let axes=showAxes(r,[0,0,0],1.1,'B');
  const sensorOrigin=body([.23,0,.73]),sensorWorld=multiply3(r,sensorMatrix);
  axes+=`<circle cx="${project(sensorOrigin)[0]}" cy="${project(sensorOrigin)[1]}" r="5" fill="#af8cff"/>`;
  // 将传感器朝向单独放在右侧，避免三个坐标系的正负标签叠在箭体上。
  axes+=`<g transform="translate(325,25)">${showAxes(sensorWorld,[0,0,0],.48,'S')}</g><text x="695" y="397" fill="#a9bad0" font-size="13" font-family="system-ui">传感器轴 S · 朝向示意</text>`;
  let thrust='';if(valid){const a=body(pivot),b=body(add(pivot,scale(dir,.85))),c=body(add(pivot,scale(dir,-.82)));
    const [bx,by]=project(b);
    thrust=arrow(a,b,'#ffd17c',4)+`<polyline points="${bx},${by} 650,165 698,165" fill="none" stroke="#ffd17c" stroke-width="1" opacity=".7"/><text x="705" y="170" fill="#ffd17c" font-size="14" font-family="system-ui">推力 +T（指令）</text>`+arrow(body(rim),c,'#ed9363',2)+text(c,'喷口 / 排气 −T','#ed9363',12);
  }
  const label=mirrored?'镜像坐标预览':valid?'箭头方向 = B+Z':'等待有效姿态 · 参考模型';
  return `<rect width="1000" height="530" fill="#0b1727"/>${grid}${world}<g opacity="${valid?1:.35}">${faces.map(f=>f.markup).join('')}${thrust}${axes}</g><text x="28" y="34" fill="#8da6bf" font-size="13" font-family="system-ui">${label}</text><text x="28" y="506" fill="#6d879e" font-size="12" font-family="system-ui">实线正方向 · 虚线负方向 · 拖动可旋转视角</text>`;
}

function createAttitudePreview({get,settings,changed=()=>{}}){
  let lastPacket=null,lastFresh=false,custom=null,axes=[1,2,3],mappingNotice='',camera=42,drag=null,azimuth=180;
  const state={mode:'live'};
  const number=id=>Number(get(id).value);
  const output=(id,text)=>{get(id).textContent=text;};
  function render(packet=lastPacket,fresh=lastFresh){
    lastPacket=packet;lastFresh=fresh;
    const sim=state.mode==='simulation',w=packet?.world;
    const measured=!!(fresh&&w?.valid&&validMatrix(w.r_bw));
    const actualMount=validMatrix(packet?.control?.mount_matrix)?packet.control.mount_matrix:IDENTITY;
    const base=sim?eulerMatrix(number('sim-roll'),number('sim-pitch'),number('sim-yaw')):measured?w.r_bw:IDENTITY;
    const originalMount=sim?IDENTITY:actualMount,map=custom||originalMount;
    const valid=sim||measured,r=custom?remapAttitude(base,originalMount,map):base;
    const cfg=settings(),manual=get('sim-gimbal-mode').value==='manual';
    let az=180,el=135;
    if(sim){
      if(manual){az=number('sim-azimuth');el=number('sim-elevation');}
      else {const result=staticGimbal(r,cfg,azimuth);az=result.azimuth;el=result.elevation;azimuth=az;get('sim-azimuth').value=fmt(az);get('sim-elevation').value=fmt(el);}
    }else if(fresh&&packet?.outputs?.servo_us?.length===2){az=(packet.outputs.servo_us[0]-500)*360/2000;el=(packet.outputs.servo_us[1]-500)*270/2000;}
    for(const id of ['sim-roll','sim-pitch','sim-yaw','sim-gimbal-mode'])get(id).disabled=!sim;
    for(const id of ['sim-azimuth','sim-elevation'])get(id).disabled=!(sim&&manual);
    for(const id of ['sim-roll','sim-pitch','sim-yaw','sim-azimuth','sim-elevation'])output(id+'-value',`${fmt(number(id))}°`);
    get('preview-mode').value=state.mode;
    output('preview-status',sim?'本地模拟 · 不发送硬件命令':!packet?'等待设备':!fresh?'连接中断 · 姿态不可用':!w?.valid?'姿态未建立':`实测跟随 · IMU${packet.control.source_id}`);
    get('preview-status').className='badge '+(sim?'simulation-badge':valid?'live':'');
    output('preview-scope',sim?'当前图形来自本地模拟输入；真实硬件控制保持独立。':'箭体使用设备姿态；TVC 使用 PWM 角度指令，没有舵机位置反馈。');
    output('preview-model-note',sim?(manual?'手动几何预览 · 不模拟机械响应':'静态 P 指向试算 · 不含 I/D、运动限速、气动力或下降轨迹'):'指令几何预览 · 不表示实物已到位');
    output('preview-azimuth',valid?`${fmt(az)}°`:'—°');output('preview-elevation',valid?`${fmt(el)}°（偏转 ${fmt(el-135)}°）`:'—°');
    const dir=gimbalDirection(az,el,cfg.azimuth_zero_deg);
    output('preview-direction',valid?vector(dir):'— / — / —');
    output('preview-tilt',valid?`${fmt(Math.acos(clamp(r[8],-1,1))/RAD)}°`:'—°');
    output('axis-map-status',mappingNotice||(custom?(determinant3(map)>0?'自定义右手系 · 仅影响预览':'自定义左手系 / 镜像 · 仅影响预览'):'沿用设备安装坐标 · 仅预览'));
    get('axis-map-status').className='hint '+(mappingNotice?'axis-error':custom&&determinant3(map)<0?'axis-warning':'');
    const raw=sim?transform3(transpose3(originalMount),[base[6],base[7],base[8]]):fresh?packet?.control_imu?.a:null;
    output('axis-preview-accel',raw?vector(transform3(map,raw)):'— / — / —');
    output('axis-matrix',map.map(v=>Math.abs(v)<.0005?'0.000':v.toFixed(3)).reduce((rows,v,i)=>rows+v.padStart(7)+(i%3===2?'\n':' '),''));
    for(let k=0;k<3;k++)output('axis-correspondence-'+k,custom?`B${'XYZ'[k]} ← ${axes[k]>0?'+':'−'}S${'XYZ'[Math.abs(axes[k])-1]}`:`B${'XYZ'[k]} ← 设备安装矩阵第 ${k+1} 行`);
    get('attitude-scene').innerHTML=sceneSvg(r,az,el,{camera,view:get('preview-camera').value,sensorMatrix:map,zero:cfg.azimuth_zero_deg,valid,mirrored:determinant3(map)<0});
  }
  get('preview-mode').addEventListener('change',()=>{state.mode=get('preview-mode').value==='simulation'?'simulation':'live';render();changed();});
  for(const id of ['sim-roll','sim-pitch','sim-yaw','sim-gimbal-mode','sim-azimuth','sim-elevation','preview-camera'])get(id).addEventListener(id==='preview-camera'||id==='sim-gimbal-mode'?'change':'input',()=>render());
  get('axis-preview-form').addEventListener('submit',event=>{
    event.preventDefault();try{const next=[0,1,2].map(k=>number('map-axis-'+k));const matrix=axisMapping(next);axes=next;custom=matrix;mappingNotice='';}catch(error){mappingNotice=error.message;}render();
  });
  get('axis-reset').addEventListener('click',()=>{custom=null;axes=[1,2,3];mappingNotice='';axes.forEach((n,k)=>get('map-axis-'+k).value=String(n));render();});
  get('sim-upright').addEventListener('click',()=>{for(const id of ['sim-roll','sim-pitch','sim-yaw'])get(id).value=0;get('sim-azimuth').value=180;get('sim-elevation').value=135;azimuth=180;render();});
  const scene=get('attitude-scene');
  scene.addEventListener('pointerdown',event=>{drag={x:event.clientX,camera};scene.setPointerCapture?.(event.pointerId);});
  scene.addEventListener('pointermove',event=>{if(drag){camera=drag.camera+(event.clientX-drag.x)*.4;render();}});
  scene.addEventListener('pointerup',()=>{drag=null;});scene.addEventListener('pointercancel',()=>{drag=null;});
  // 单独的图形刷新没有硬件副作用；实测输入仍只来自主控制台传入的遥测包。
  return {render,get mode(){return state.mode;}};
}

return {createAttitudePreview};})();


const $=id=>document.getElementById(id);
function makeSession(){
  try{return Array.from(crypto.getRandomValues(new Uint8Array(10)),byte=>byte.toString(16).padStart(2,'0')).join('');}
  catch{return `${Date.now().toString(16)}${Math.random().toString(16).slice(2)}`.slice(0,20).padEnd(8,'0');}
}
const S={mode:null,port:null,reader:null,writer:null,ws:null,token:'',session:makeSession(),seq:0,boot:'',epoch:0,rev:0,tele:null,lastRx:0,pending:new Map(),generation:0,heartbeat:false,view:'auto',connecting:false,writeChain:Promise.resolve(),faultShown:'',deviceFirmware:'',action:0,operation:null,stopping:0,controlChain:Promise.resolve(),healthPending:false,lastKeepalive:0,autoFanWanted:false};
const ESC={stop_us:1500,min_us:1300,max_us:1700,hold_ms:3000,hz:50,confirmed:true};
// 预览模块只有 DOM 与数学接口，没有 command()/串口/WebSocket 的访问权限。
const attitudePreview=createAttitudePreview({get:$,settings:()=>settingsFromPacket(S.tele)||{kp_x:.6,kp_y:.6,sign_x:1,sign_y:1,max_tilt_deg:15,azimuth_zero_deg:180}});
Object.assign(S,{trend:[],trendBoot:'',trendReference:0,pidDirty:false,autoThrottleWanted:false,faultSnapshot:null,warningShown:''});
const angleToUs=servoAngleToUs;
const usToAngle=(us,channel)=>servoUsToAngle(us,channel).toFixed(1);
const fresh=()=>!!S.tele&&performance.now()-S.lastRx<TRANSPORT_GRACE_MS;
const ready=()=>fresh()&&!S.tele.fault&&!S.tele.calibrating&&S.tele.control_imu?.valid;
const outputsActive=()=>!!(S.tele&&(S.tele.outputs.servo_enabled||S.tele.outputs.esc_armed||S.tele.outputs.esc_arming));

function notice(message,type=''){$('notice').textContent=message;$('notice').className=`notice ${type}`.trim();}
function describeReason(reason){if(reason==='FAULT_REQUIRES_RESTART')return `重新校准被当前故障拒绝；原始 fault：${S.tele?.fault||'未知，请查看原始故障码区域'}`;return ({ELEVATION_LIMIT:'俯仰限位为中位 ±50°（85～185°）',THROTTLE_NOT_READY:'自动油门尚未就绪',AUTO_THROTTLE_ACTIVE:'自动油门正在接管电机',THROTTLE_CONFIG:'自动油门参数无效',IMU_NOT_FOUND:'目标 IMU 未检测到，已保留原数据源',INVALID_IMU_SOURCE:'IMU 数据源编号无效',CONTROLLER_BUSY:'已有另一个控制页面连接，请先断开它',AUTH_REQUIRED:'无线认证失败',SESSION_MISMATCH:'连接会话已失效',STALE_COMMAND:'设备数据过期，请重新连接',MODE_LOCKED:'当前不在对应控制模式',NOT_READY:'设备尚未准备好',STOP_OUTPUTS_FIRST:'请先停止全部输出',WORLD_NOT_READY:'世界坐标系尚未建立',CONFIG_MISMATCH:'电调配置已变化，请重试',OLD_EPOCH:'控制状态已变化，请重试'})[reason]||reason;}

function acceptPacket(p){
  if(!p||p.v!==VERSION||p.session!==S.session)return;
  if(p.type==='ack'){
    if(S.boot&&p.boot!==S.boot)return;
    if(typeof p.boot==='string')S.boot=p.boot;
    if(Number.isInteger(p.epoch))S.epoch=Math.max(S.epoch,p.epoch);
    if(Number.isInteger(p.config_rev))S.rev=Math.max(S.rev,p.config_rev);
    const waiter=S.pending.get(p.request_seq);
    if(waiter){clearTimeout(waiter.timer);S.pending.delete(p.request_seq);p.ok?waiter.resolve(p):waiter.reject(new Error(describeReason(p.reason)));}
  }else if(isTelemetry(p)){
    if(S.boot&&p.boot!==S.boot)return;
    S.boot=p.boot;S.epoch=Math.max(S.epoch,p.epoch);S.rev=Math.max(S.rev,p.config_rev);S.tele=p;S.lastRx=performance.now();recordTrend(p);
    if((p.fault||p.warning)&&(!S.faultSnapshot||S.faultSnapshot.packet.boot!==p.boot||S.faultSnapshot.packet.fault!==p.fault||S.faultSnapshot.packet.warning!==p.warning||S.faultShown!==p.fault||S.warningShown!==p.warning))S.faultSnapshot={packet:JSON.parse(JSON.stringify(p)),received:new Date().toISOString()};
    S.warningShown=p.warning;
    if(p.calibrating||p.operating_mode!=='ATTITUDE')resetAutoMotor();
    if(p.fault&&p.fault!==S.faultShown){S.faultShown=p.fault;resetAutoMotor();cancelActions();S.heartbeat=false;notice(faultDescription(p),'error');}
    if(!p.fault)S.faultShown='';
  }
  render();
}

function feedText(text){new LineDecoder().push(text,acceptPacket,()=>notice('收到损坏的数据帧','error'));}
const abort=()=>Object.assign(new Error('操作已取消'),{name:'AbortError'});
function ticket(){return {generation:S.generation,action:S.action,session:S.session};}
function check(t){if(t&&(t.generation!==S.generation||t.action!==S.action||t.session!==S.session||!S.mode||document.hidden))throw abort();}
function cancelActions(){
  ++S.action;S.operation=null;clearTimeout(servoTimer);clearTimeout(motorTimer);clearTimeout(autoMotorTimer);
  for(const [seq,p] of S.pending)if(p.guard){clearTimeout(p.timer);S.pending.delete(seq);p.reject(abort());}
}
async function operation(kind,fn){
  if(S.operation||S.stopping)throw abort();
  const t=ticket();S.operation={kind,t};render();
  try{check(t);return await fn(t);}catch(error){if(t.generation===S.generation&&t.action===S.action)S.heartbeat=outputsActive();throw error;}finally{if(S.operation?.t===t)S.operation=null;render();}
}
function rawWrite(frame,guard){
  const generation=S.generation,writer=S.writer,ws=S.ws,mode=S.mode;
  const send=()=>{if(generation!==S.generation||mode!==S.mode)throw abort();check(guard);return typeof frame==='function'?frame():frame;};
  if(mode==='wifi'&&ws?.readyState===WebSocket.OPEN){try{ws.send(send());return Promise.resolve();}catch(e){return Promise.reject(e);}}
  if(mode==='serial'&&writer){S.writeChain=S.writeChain.catch(()=>{}).then(()=>writer.write(new TextEncoder().encode(send())));return S.writeChain;}
  return Promise.reject(new Error('设备未连接'));
}
function sendCommand(cmd,data={},timeout=1800,guard){
  const seq=++S.seq,session=S.session,boot=S.boot;
  return new Promise((resolve,reject)=>{
    const timer=setTimeout(()=>{S.pending.delete(seq);reject(new Error('设备响应超时'));},timeout);
    S.pending.set(seq,{resolve,reject,timer,guard});
    rawWrite(()=>{
      if(!S.pending.has(seq)||S.session!==session)throw abort();
      const packet={v:VERSION,type:'command',seq,session,cmd,...data};
      if(cmd==='hello'){if(S.token)packet.token=S.token;}else{
        packet.boot=boot;
        if(S.tele)packet.device_t_us=Math.round(S.tele.t_us+(performance.now()-S.lastRx)*1000);
      }
      return encodeFrame(packet);
    },guard).catch(error=>{clearTimeout(timer);S.pending.delete(seq);reject(error);});
  });
}
const immediate=new Set(['hello','heartbeat','keepalive','stop','servo_disable','esc_stop','auto_disable','auto_motor_disable','tilt_throttle_disable']);
function command(cmd,data={},timeout=1800,guard){
  if(immediate.has(cmd))return sendCommand(cmd,data,timeout,guard);
  const t=guard||ticket();
  const work=S.controlChain.catch(()=>{}).then(async()=>{
    check(t);const result=await sendCommand(cmd,{...data,epoch:S.epoch},timeout,t);check(t);return result;
  });
  S.controlChain=work.catch(()=>{});return work;
}
async function run(label,fn){try{await fn();notice(label,'success');}catch(error){if(error.name!=='AbortError')notice(error.message||String(error),'error');}finally{render();}}
const delay=ms=>new Promise(resolve=>setTimeout(resolve,ms));
async function waitTelemetry(test=()=>true,timeout=5000,guard){
  const generation=S.generation,end=performance.now()+timeout;
  while(performance.now()<end){if(generation!==S.generation)throw abort();check(guard);if(fresh()&&test(S.tele))return S.tele;await delay(50);}
  throw new Error('等待设备状态超时');
}
function resetSession(){
  resetAutoMotor();cancelActions();S.session=makeSession();S.seq=0;S.boot='';S.epoch=S.rev=0;S.tele=null;S.lastRx=0;S.faultShown='';S.warningShown='';
  S.writeChain=Promise.resolve();S.controlChain=Promise.resolve();S.healthPending=false;S.lastKeepalive=0;
}
function faultDescription(p){
  const m=p.control_imu||{};
  const link=p.control.source_id===2?`IMU2 本地 I2C，读取错误 ${m.read_errors??0}，丢帧 ${m.missed??0}`:`IMU1 串口回复 ${m.uart_replies??0}，串口坏帧 ${m.uart_bad_frames??0}，串口超时 ${m.uart_timeouts??0}`;
  const detail=`${link}；有效样本 ${m.calibration_samples??'—'}/300，剔除 ${m.calibration_rejected??'—'}，读取错误累计 ${m.read_errors??'—'}`;
  if(!p.fault&&p.warning==='HEARTBEAT_TIMEOUT')return `心跳超时告警：本项不停止输出，也不取消控制操作；心跳恢复后自动解除。最近心跳年龄 ${p.heartbeat_age_ms??'—'}ms。`;
  if(p.fault==='HEARTBEAT_TIMEOUT')return '当前主控将心跳超时作为停机故障处理。需要上传2.4.2主控，才能改为仅告警。';
  if(p.fault==='PID_INVALID')return 'PID 输入、参数或控制周期检查未通过，全部输出已停止。可能涉及控制间隔超过50ms、姿态输入非有限或指令范围异常；此总故障码还不能区分具体触发项。';
  if(p.fault==='CALIBRATION_DATA')return `校准未通过（CALIBRATION_DATA）：${detail}。请保持静止，检查主IMU接线后重新建立坐标系；样本足够时还需检查数据波动。`;
  if(p.fault==='IMU_TIMEOUT')return p.control.source_id===2?`IMU2 在历史窗口内无有效新样本，输出已停止。检查本地 SDA9、SCL10、INT8、AD0 高电平及供电，恢复后重新校准。`:`IMU1 在历史窗口内无有效新样本，输出已停止。检查采集板供电、TX6→RX16、TX17→RX7及共地，恢复后重新校准。${detail}`;
  if(p.fault==='IMU_RESTART')return '采集板已重启（IMU_RESTART），输出已停止。请保持竖直静止并重新校准。';
  if(p.fault==='MOUNT_CALIBRATION')return '安装校准未通过：箭头朝上、箭体保持竖直，MPU6050 板面沿箭体竖直后重新校准。';
  if(p.fault==='ATTITUDE_INVALID')return '姿态数据无效，输出已停止。请检查当前 IMU 的数据，恢复后重新校准。';
  if(p.fault==='THROTTLE_INVALID')return '自动油门计算或控制周期异常，全部输出已停止。';
  if(p.fault==='PREPOSITION_MOVED')return '启动准备期间姿态变化过大或姿态无效，输出已停止；请固定台架后重新准备。';
  if(p.fault==='PREPOSITION_TIMEOUT')return '舵机预定位超时，输出已停止；请检查姿态、舵机及限速。';
  if(p.fault==='PWM_INIT')return '舵机 PWM 输出失败，已停止控制。请检查开发板配置与引脚占用。';
  return `设备故障：${p.fault}`;
}
function displayedFault(){return S.tele&&(S.tele.fault||S.tele.warning)?S.tele:S.tele?null:S.faultSnapshot?.packet??null;}
function renderFault(){
  const p=displayedFault(),current=!!S.tele;
  $('device-fault-code').textContent=p?.fault||p?.warning||(current?'无故障':'等待连接');
  $('device-fault-code').className=p?(p.fault?'fault-active':'fault-warning'):'';
  $('device-fault-description').textContent=p?faultDescription(p):current?'设备当前 fault 为空。':'连接后持续显示设备 fault，不受操作提示覆盖。';
  $('device-fault-age').textContent=p?`${current?(fresh()?'当前设备回传':'连接数据已过期'):'保留的历史故障快照'} · 固件 ${p.firmware} · IMU${p.control.source_id} · ${p.state}`:current?'当前设备回传正常':'尚未收到设备遥测';
  $('copy-fault').disabled=!p;
  if(!p){$('fault-copy-text').hidden=true;$('fault-copy-text').value='';$('fault-copy-status').textContent='复制后可直接粘贴到聊天中。';}
}
async function copyFault(){
  const p=displayedFault();if(!p?.fault&&!p?.warning)return;
  const data={fault:p.fault,warning:p.warning||'',heartbeat_age_ms:p.heartbeat_age_ms,firmware:p.firmware,boot:p.boot,state:p.state,operating_mode:p.operating_mode,control_source:p.control.source_id,t_us:p.t_us,
    snapshot:!S.tele,telemetry_fresh:!!S.tele&&fresh(),first_received:S.faultSnapshot?.received,
    imu:p.control_imu,mount:{ready:p.control.mount_ready,yaw_deg:p.control.mount_yaw_deg,sensor_to_body:p.control.mount_matrix},world_valid:p.world.valid,pid:p.control.pid,outputs:p.outputs,
    note:p.fault?'这是故障后的遥测；PID复位后的dt可能为0，不能据此认定原始触发原因。':'当前为告警；控制不会因这项告警中断。'};
  const text=JSON.stringify(data,null,2),field=$('fault-copy-text');field.value=text;
  try{if(!navigator.clipboard?.writeText)throw Error('CLIPBOARD_UNAVAILABLE');await navigator.clipboard.writeText(text);field.hidden=true;$('fault-copy-status').textContent='故障信息已复制，可直接粘贴到聊天中。';}
  catch{field.hidden=false;field.focus?.();field.select?.();$('fault-copy-status').textContent='浏览器未允许直接复制，请选中文本框内容，按 Ctrl+C 复制。';}
}

async function beginSession(){
  await command('hello',{},2500);
  await waitTelemetry(()=>true,2500);
  notice('连接成功，设备输出保持停止。','success');
  await switchDeviceMode(S.view);
}
async function connectUsb(){
  if(S.connecting||S.mode)return;
  if(!('serial'in navigator)){notice('当前浏览器不支持USB串口，请使用Chrome或Edge。','error');return;}
  S.connecting=true;render();
  try{
    const port=await navigator.serial.requestPort();await port.open({baudRate:460800});
    resetSession();S.mode='serial';S.port=port;S.writer=port.writable.getWriter();S.generation++;
    const decoder=new LineDecoder();
    S.reader=port.readable.getReader();const reader=S.reader,generation=S.generation;
    (async()=>{try{while(S.mode==='serial'&&generation===S.generation){const {value,done}=await reader.read();if(done||generation!==S.generation)break;decoder.push(value,acceptPacket,()=>notice('USB数据帧校验失败','error'));}}catch(error){if(generation===S.generation)notice(`USB连接中断：${error.message}`,'error');}finally{if(generation===S.generation)void disconnect(false);}})();
    await beginSession();
  }catch(error){await closeTransport();notice(`USB连接失败：${error.message}`,'error');}
  finally{S.connecting=false;render();}
}
async function connectWifi(){
  if(location.hostname!=='192.168.4.1'){location.href='http://192.168.4.1/?ui=2.4.3';return;}
  if(S.connecting||S.mode)return;S.connecting=true;render();
  try{
    const response=await fetch('/api/device',{cache:'no-store'});if(!response.ok)throw new Error(`设备信息 HTTP ${response.status}`);
    const info=await response.json();S.deviceFirmware=typeof info.firmware==='string'?info.firmware:'版本未知';render();if(info.protocol!==VERSION)throw new Error(`网页协议 ${VERSION} 与主控协议 ${info.protocol} 不一致；当前固件 ${S.deviceFirmware}，请上传本目录的主控工程`);
    resetSession();S.token=info.token;const ws=new WebSocket(info.ws_url);S.ws=ws;
    await new Promise((resolve,reject)=>{const timer=setTimeout(()=>reject(new Error('无线连接超时')),3000);ws.onopen=()=>{clearTimeout(timer);resolve();};ws.onerror=()=>{clearTimeout(timer);reject(new Error('WebSocket连接失败'));};});
    S.mode='wifi';S.generation++;
    const generation=S.generation;ws.onmessage=event=>{if(generation===S.generation)feedText(String(event.data));};
    ws.onclose=()=>{if(generation===S.generation&&S.mode==='wifi'){notice('Wi-Fi控制连接已断开','error');void disconnect(false);}};
    await beginSession();
  }catch(error){await closeTransport();notice(`Wi-Fi连接失败：${error.message}`,'error');}
  finally{S.connecting=false;render();}
}
async function closeTransport(){
  const {mode,reader,writer,port,ws}=S;resetAutoMotor();
  cancelActions();S.mode=null;S.heartbeat=false;S.tele=null;S.lastRx=0;S.boot='';S.generation++;
  S.reader=S.writer=S.port=S.ws=null;S.writeChain=Promise.resolve();S.controlChain=Promise.resolve();
  for(const pending of S.pending.values()){clearTimeout(pending.timer);pending.reject(abort());}S.pending.clear();
  try{if(mode==='serial'&&reader){await reader.cancel();reader.releaseLock();}}catch{}
  try{if(mode==='serial'&&writer)writer.releaseLock();}catch{}
  try{if(mode==='serial'&&port)await port.close();}catch{}
  try{if(ws&&ws.readyState<2)ws.close();}catch{}render();
}
async function disconnect(sendStop=true){
  resetAutoMotor();cancelActions();S.heartbeat=false;++S.stopping;render();
  try{if(sendStop&&S.mode&&S.boot)try{await command('stop',{},600);}catch{}await closeTransport();notice('已断开，已请求设备停止输出。');}
  finally{--S.stopping;render();}
}
async function stopPart(cmd){
  cancelActions();if(cmd==='stop')S.heartbeat=false;
  const generation=S.generation,action=S.action;++S.stopping;render();
  try{
    if(S.mode&&S.boot){const ack=await command(cmd);await waitTelemetry(p=>p.epoch>=ack.epoch&&
      ((cmd==='stop'||cmd==='auto_disable')?!p.outputs.servo_enabled&&!p.outputs.esc_armed&&!p.outputs.esc_arming:
       (cmd==='esc_stop'||cmd==='auto_motor_disable'||cmd==='tilt_throttle_disable')?!p.outputs.esc_armed&&!p.outputs.esc_arming:!p.outputs.servo_enabled),1500);}
  }finally{--S.stopping;if(generation===S.generation&&action===S.action)S.heartbeat=cmd!=='stop'&&outputsActive();render();}
}
async function stopAll(){resetAutoMotor();await stopPart('stop');}
async function stopAutomatic(){resetAutoMotor();await stopPart('auto_disable');}

async function switchDeviceMode(view){
  if(!S.mode||!S.tele)return;
  const target=view==='auto'?'ATTITUDE':'MANUAL';if(S.tele.operating_mode===target)return;
  resetAutoMotor();cancelActions();S.heartbeat=false;await operation('mode',async t=>{
    const ack=await command('set_mode',{mode:target},3500,t);
    await waitTelemetry(p=>p.epoch>=ack.epoch&&p.operating_mode===target,2000,t);
  });
  if(target==='ATTITUDE')notice('正在静止校准，约需3秒。','success');else notice('已进入手动控制。','success');
}
async function showPage(view){
  S.view=view;$('page-auto').hidden=view!=='auto';$('page-manual').hidden=view!=='manual';$('tab-auto').classList.toggle('active',view==='auto');$('tab-manual').classList.toggle('active',view==='manual');render();
  if(S.mode)await run(view==='auto'?'已切换到自动回正。':'已切换到手动控制。',()=>switchDeviceMode(view));
}

function servoValues(){return {x:angleToUs($('servo-x').value,0),y:angleToUs($('servo-y').value,1)};}
async function sendServo(){if(!S.tele?.outputs.servo_enabled)return;await command('servo',{epoch:S.epoch,...servoValues()});}
let servoTimer=0;
function queueServo(){clearTimeout(servoTimer);servoTimer=setTimeout(()=>run('舵机角度已更新。',sendServo),80);}
async function enableServos(){return operation('servo',async t=>{S.heartbeat=true;await command('heartbeat',{},1800,t);check(t);const ack=await command('servo_enable',{},1800,t);await waitTelemetry(p=>p.epoch>=ack.epoch&&p.outputs.servo_enabled,2000,t);$('servo-x').value=180;$('servo-y').value=135;updateLabels();});}
async function disableServos(){await stopPart('servo_disable');}

const motorUs=()=>Math.round(1500+Number($('motor-speed').value)*2);
async function startMotor(){return operation('motor',async t=>{
  if(!$('prop-confirm').checked)throw new Error('请先确认桨叶已经拆除');
  validateEscConfig(ESC);S.heartbeat=true;await command('heartbeat',{},1800,t);check(t);
  if(!S.tele?.outputs.esc_armed){
    await command('esc_config',ESC,1800,t);check(t);
    const ack=await command('esc_arm',{config_rev:S.rev},1800,t);
    await waitTelemetry(p=>p.epoch>=ack.epoch&&p.outputs.esc_armed,5200,t);
  }
  await command('esc',{us:motorUs()},1800,t);
});}
async function stopMotor(){S.autoThrottleWanted=false;$('tilt-throttle-enable').checked=false;await stopPart('esc_stop');}
const throttleInputs={'throttle-base':'base_pct','throttle-gain':'gain_pct_deg','throttle-max':'max_pct','throttle-deadband':'deadband_deg','throttle-ramp':'ramp_pct_s'};
function throttleSettings(){return validateTiltThrottle(Object.fromEntries(Object.entries(throttleInputs).map(([id,key])=>[key,Number($(id).value)])));}
async function toggleTiltThrottle(){
  const wanted=$('tilt-throttle-enable').checked;
  if(wanted&&S.autoFanWanted)throw Error('请先关闭独立手动调速');
  S.autoThrottleWanted=wanted;
  if(!wanted)await stopPart('tilt_throttle_disable');
}
async function startAutomatic(){return operation('auto',async t=>{
  const coupled=S.autoThrottleWanted,config=coupled?throttleSettings():null;
  S.heartbeat=true;await command('heartbeat',{},1800,t);check(t);
  try{
    if(coupled){
      if(!S.tele?.control?.tilt_throttle)throw Error('当前固件不支持倾斜自动油门，请更新主控');
      if(typeof S.tele?.control?.preposition_active!=='boolean')throw Error('主控固件不支持电机启动前预定位，请更新主控');
      await command('tilt_throttle_config',config,1800,t);check(t);
      await command('auto_motor_enable',{},1800,t);check(t);
      validateEscConfig(ESC);await command('esc_config',ESC,1800,t);check(t);
      const arm=await command('esc_arm',{config_rev:S.rev},1800,t);
      await waitTelemetry(p=>{if(p.fault)throw Error(faultDescription(p));return p.epoch>=arm.epoch&&p.outputs.esc_armed;},23000,t);check(t);
    }
    const ack=await command('auto_enable',coupled?{tilt_throttle:true}:{},1800,t);
    await waitTelemetry(p=>p.epoch>=ack.epoch&&p.control?.auto_enabled&&(!coupled||p.control?.tilt_throttle?.enabled),2000,t);
  }catch(error){
    if(coupled&&t.generation===S.generation&&t.action===S.action)await command('tilt_throttle_disable').catch(()=>{});
    throw error;
  }
});}
async function recalibrate(){resetAutoMotor();cancelActions();S.heartbeat=false;return operation('calibrate',t=>command('calibrate',{},3500,t));}
async function switchImuSource(source){
  if(!S.mode||!fresh())throw new Error('请先连接主控');
  await stopAll();
  await operation('source',async t=>{
    const ack=await command('select_imu',{source},2500,t);
    await waitTelemetry(p=>p.epoch>=ack.epoch&&p.control.source_id===source,2500,t);
  });
}
let motorTimer=0;
function queueMotor(){clearTimeout(motorTimer);if(S.tele?.outputs.esc_armed)motorTimer=setTimeout(()=>run('电机速度已更新。',()=>command('esc',{epoch:S.epoch,us:motorUs()})),100);}

let autoMotorTimer=0;
const autoMotorUs=()=>Math.round(1500+Number($('auto-motor-speed').value)*2);
function resetAutoMotor(){S.autoFanWanted=false;S.autoThrottleWanted=false;clearTimeout(autoMotorTimer);$('tilt-throttle-enable').checked=false;$('auto-motor-enable').checked=false;$('auto-motor-speed').value=0;}
async function toggleAutoMotor(){
  const wanted=$('auto-motor-enable').checked;S.autoFanWanted=wanted;
  if(wanted&&S.autoThrottleWanted){S.autoFanWanted=false;throw Error('请先关闭倾斜自动油门');}
  if(!wanted){resetAutoMotor();await stopPart('auto_motor_disable');return;}
  $('auto-motor-speed').value=0;
  try{await operation('fan-optin',async t=>{
    S.heartbeat=true;await command('heartbeat',{},1800,t);check(t);
    const ack=await command('auto_motor_enable',{},1800,t);
    await waitTelemetry(p=>p.epoch>=ack.epoch&&p.control?.auto_motor_allowed,2000,t);
  });}catch(error){if(error.name!=='AbortError')resetAutoMotor();throw error;}
}
async function startAutoMotor(){return operation('auto-motor',async t=>{
  if(S.autoThrottleWanted||S.tele?.control?.tilt_throttle?.enabled)throw Error('倾斜自动油门已接管，不能同时手动调速');
  if(!S.autoFanWanted||!S.tele?.control?.auto_motor_allowed||S.tele.operating_mode!=='ATTITUDE')throw new Error('请先勾选启用电机调速');
  if(typeof S.tele?.control?.preposition_active!=='boolean')throw Error('主控固件不支持电机启动前预定位，请更新主控');
  validateEscConfig(ESC);S.heartbeat=true;await command('heartbeat',{},1800,t);check(t);
  try{
    if(!S.tele.outputs.esc_armed){
      await command('esc_config',ESC,1800,t);check(t);
      const ack=await command('esc_arm',{config_rev:S.rev},1800,t);
      await waitTelemetry(p=>{if(p.fault)throw Error(faultDescription(p));return p.epoch>=ack.epoch&&p.outputs.esc_armed&&p.control?.auto_motor_allowed;},23000,t);
    }
    check(t);if(!S.autoFanWanted)throw abort();await command('esc',{us:autoMotorUs()},1800,t);
  }catch(error){
    if(t.generation===S.generation&&t.action===S.action)await command('esc_stop').catch(()=>{});
    throw error;
  }
});}
async function sendAutoMotor(t=ticket()){
  if(!S.autoFanWanted||!S.tele?.control?.auto_motor_allowed||!S.tele?.outputs.esc_armed)return;
  check(t);await command('esc',{us:autoMotorUs()},1800,t);
}
function queueAutoMotor(){clearTimeout(autoMotorTimer);const t=ticket();if(S.tele?.outputs.esc_armed)autoMotorTimer=setTimeout(()=>run('风扇速度已更新。',()=>sendAutoMotor(t)),100);}

function updateLabels(){$('servo-x-value').textContent=`${$('servo-x').value}°`;$('servo-y-value').textContent=`${$('servo-y').value}°`;$('motor-speed-value').textContent=`${$('motor-speed').value}%`;$('auto-motor-speed-value').textContent=`${$('auto-motor-speed').value}%`;}
function renderImu2(packet,isFresh){
  const view=imu2Display(packet?.imu2,packet?.t_us,isFresh);
  $('imu2-status').textContent=view.status;$('imu2-status').classList.toggle('live',view.live);
  $('imu2-pitch').textContent=view.pitch;$('imu2-roll').textContent=view.roll;
  $('imu2-temp').textContent=view.temperature;$('imu2-rate').textContent=view.rate;
  $('imu2-accel').textContent=view.live?formatVector(packet.imu2?.a_g,3):'— / — / —';
  $('imu2-gyro').textContent=view.live?formatVector(packet.imu2?.g_dps,2):'— / — / —';
}
// IMU1（采集板 0x68）始终显示为校验卡片：它是否参与 PID 由固件 control.source_id 决定。
function renderImu1(packet,isFresh){
  const s=packet?.imus?.[0],live=!!(isFresh&&s?.valid);
  const status={OK:'正常',STALE:'数据过期',WAIT_SAMPLE:'等待样本',WAIT_C3:'等待采集板',C3_NOT_FOUND:'采集板未找到IMU',NOT_FOUND:'未检测到'}[s?.status]||'异常';
  $('imu1-status').textContent=!packet?'等待连接':!isFresh?'数据过期':status;
  $('imu1-status').classList.toggle('live',live);
  $('imu1-age').textContent=live?`${Math.max(0,(packet.t_us-s.sample_t_us)/1000).toFixed(1)} ms`:'—';
  $('imu1-rate').textContent=live&&Number.isFinite(s.sample_hz)?`${s.sample_hz.toFixed(1)} Hz`:'—';
  $('imu1-temp').textContent=live&&Number.isFinite(s.temperature)?`${s.temperature.toFixed(1)} ℃`:'— ℃';
  $('imu1-uart').textContent=live?`${s.uart_replies??0} / ${s.uart_bad_frames??0}`:'— / —';
  $('imu1-accel').textContent=live?formatVector(s.a,3):'— / — / —';
  $('imu1-gyro').textContent=live?formatVector(s.g,2):'— / — / —';
  $('imu1-role').textContent=packet?.control?.source_id===1?'当前控制源；原始轴数据（安装矩阵变换前）':'备用监测；不参与当前 PID';
}
const pidInputs={'kp-x':'kp_x','ki-x':'ki_x','kd-x':'kd_x','kp-y':'kp_y','ki-y':'ki_y','kd-y':'kd_y','max-tilt':'max_tilt_deg','azimuth-zero':'azimuth_zero_deg','sign-x':'sign_x','sign-y':'sign_y'};
function formatVector(values,digits){return Array.isArray(values)&&values.length===3?values.map(v=>Number.isFinite(v)?v.toFixed(digits):'—').join(' / '):'— / — / —';}
function settingsFromPacket(p){const c=p?.control,pid=c?.pid;if(!pid?.x||!pid?.y)return null;return {kp_x:pid.x.kp,ki_x:pid.x.ki,kd_x:pid.x.kd,kp_y:pid.y.kp,ki_y:pid.y.ki,kd_y:pid.y.kd,max_tilt_deg:pid.output_limit_deg,azimuth_zero_deg:c.azimuth_zero_deg,sign_x:c.sign_x,sign_y:c.sign_y};}
async function applyPid(){
  const settings=validatePidSettings(Object.fromEntries(Object.entries(pidInputs).map(([id,key])=>[key,Number($(id).value)])));
  await operation('pid',async t=>{
    const ack=await command('pid_config',settings,2000,t);
    await waitTelemetry(p=>p.epoch>=ack.epoch&&Object.entries(settings).every(([key,value])=>Math.abs(settingsFromPacket(p)?.[key]-value)<.0001),2500,t);
    S.pidDirty=false;
  });
}
function renderDetails(p,isFresh,connected,busy){
  const sensor=isFresh?p?.control_imu:null;
  const source=p?.control?.source_id??2;
  $('current-imu-title').textContent=source===2?'IMU2 · 主控本地 I2C · 0x69':'IMU1 · SuperMini · 0x68';
  $('current-error-label').textContent=source===2?'读取错误 / 丢帧':'读取错误 / 串口超时';
  $('primary-gyro').textContent=formatVector(sensor?.g,2);$('primary-accel').textContent=formatVector(sensor?.a,3);
  $('primary-temp').textContent=Number.isFinite(sensor?.temperature)?`${sensor.temperature.toFixed(1)} ℃`:'— ℃';
  $('primary-errors').textContent=sensor?`${sensor.read_errors??0} / ${(source===2?sensor.missed:sensor.uart_timeouts)??0}`:'— / —';
  $('primary-age').textContent=sensor?`${((p.t_us-sensor.sample_t_us)/1000).toFixed(1)} ms`:'等待数据';
  $('primary-rate').textContent=sensor&&Number.isFinite(sensor.sample_hz)?`${sensor.sample_hz.toFixed(1)} Hz`:'— Hz';
  $('primary-sample-age').textContent=sensor?`${Math.max(0,(p.t_us-sensor.sample_t_us)/1000).toFixed(1)} ms`:'— ms';
  $('primary-history').textContent=sensor?({LIVE:'实时',PREDICTED:'历史估算',HELD:'历史保持',UNAVAILABLE:'已失效'})[sensor.history_state]||'—':'—';
  $('primary-interface').textContent=sensor?`${sensor.source==='LOCAL_I2C_IMU2'?'LOCAL_I2C':'SUPERMINI_UART'} / 0x${Number(sensor.address).toString(16)}`:'—';
  const pid=isFresh?p?.control?.pid:null;
  for(const axis of ['x','y'])for(const [cell,key] of [['p','p_deg'],['i','i_deg'],['d','d_deg'],['out','output_deg']])$('pid-'+axis+'-'+cell).textContent=Number.isFinite(pid?.[axis]?.[key])?pid[axis][key].toFixed(2):'—';
  $('pid-interval').textContent=Number.isFinite(pid?.dt_s)?`${(pid.dt_s*1000).toFixed(1)} ms`:'— ms';
  $('pid-limit-state').textContent=!pid?'等待':!p.control.auto_enabled?'未启动':pid.limited?'输出受限 · 限制积分':'正常';
  const settings=settingsFromPacket(p);
  if(isFresh&&settings&&!S.pidDirty&&document.activeElement?.closest('#pid-form')===null)for(const [id,key] of Object.entries(pidInputs))$(id).value=settings[key];
  $('apply-pid').disabled=!(connected&&isFresh&&!outputsActive()&&!p.calibrating&&!busy&&settings);
  $('pid-config-state').textContent=!connected?'连接设备后读取实际参数':!isFresh?'等待设备数据':S.pidDirty?'有待应用的修改':settings?'已读取设备实际参数':'固件未提供 PID 参数';
  drawTrend(isFresh);
}
function recordTrend(p){
  if(S.trendBoot!==p.boot||S.trendReference!==p.world.reference_id){S.trend=[];S.trendBoot=p.boot;S.trendReference=p.world.reference_id;}
  if(!p.world.valid||!p.control.tilt_valid||!p.control.error_deg?.every(Number.isFinite))return;
  const time=p.t_us/1e6,last=S.trend.at(-1);if(last&&time-last.t<.08)return;
  if(last&&time-last.t>.75)S.trend.push({t:time-.001,x:NaN,y:NaN});
  S.trend.push({t:time,x:p.control.error_deg[1],y:p.control.error_deg[0]});
  while(S.trend.length&&S.trend[0].t<time-30)S.trend.shift();
}
function drawTrend(isFresh){
  const canvas=$('trend-chart'),rect=canvas.getBoundingClientRect();if(rect.width<1)return;
  const dpr=Math.min(window.devicePixelRatio||1,2),w=Math.round(rect.width),h=210;
  if(canvas.width!==Math.round(w*dpr)||canvas.height!==Math.round(h*dpr)){canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);}
  const ctx=canvas.getContext('2d');ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,w,h);
  const left=42,right=w-12,top=12,bottom=h-25;
  const scale=Math.max(5,Math.ceil(Math.max(0,...S.trend.flatMap(s=>[s.x,s.y]).filter(Number.isFinite).map(Math.abs))/5)*5);
  ctx.font='11px system-ui';ctx.lineWidth=1;ctx.textAlign='right';
  for(let i=-2;i<=2;i++){let y=top+(2-i)*(bottom-top)/4;ctx.strokeStyle=i===0?'#456075':'#25374b';ctx.beginPath();ctx.moveTo(left,y);ctx.lineTo(right,y);ctx.stroke();ctx.fillStyle='#8099b2';ctx.fillText(`${i*scale/2}°`,left-8,y+4);}
  ctx.textAlign='center';for(let i=0;i<=3;i++)ctx.fillText(`${i*10-30}s`,left+(right-left)*i/3,h-5);
  const end=S.trend.at(-1)?.t??0;
  for(const [key,color] of [['x','#4ce0ca'],['y','#78a7ff']]){ctx.beginPath();ctx.strokeStyle=color;ctx.lineWidth=2;let started=false;for(const point of S.trend){if(!Number.isFinite(point[key])){started=false;continue;}const x=left+(1+(point.t-end)/30)*(right-left),y=top+(1-point[key]/scale)*(bottom-top)/2;if(started)ctx.lineTo(x,y);else{ctx.moveTo(x,y);started=true;}}ctx.stroke();}
  const history=S.tele?.control_imu?.history_state,name=`IMU${S.tele?.control?.source_id??2}`;
  $('chart-state').textContent=!S.trend.length?'等待真实设备数据':!isFresh?'连接中断 · 保留已接收曲线':history==='PREDICTED'?`${name} · 历史角速度估算（非新测量）`:history==='HELD'?`${name} · 保持历史估计，等待新样本`:`${name} · 倾斜角 °`;
}
function render(){
  renderFault();
  const firmwareLabel=$('firmware-version');if(firmwareLabel)firmwareLabel.textContent=S.tele?.firmware||S.deviceFirmware||'未连接';
  const p=S.tele,isFresh=fresh(),connected=!!S.mode,ok=ready(),manual=p?.operating_mode==='MANUAL',attitude=p?.operating_mode==='ATTITUDE';
  $('connection-status').textContent=S.connecting?'连接中':connected?(isFresh?(S.mode==='wifi'?'Wi-Fi 已连接':'USB 已连接'):'数据中断'):'未连接';$('connection-status').classList.toggle('live',connected&&isFresh);
  $('connect-wifi').textContent=location.hostname==='192.168.4.1'?'连接 Wi-Fi':'打开设备页面';
  $('connect-wifi').disabled=connected||S.connecting;$('connect-usb').disabled=connected||S.connecting;$('disconnect').disabled=!connected;$('stop-all').disabled=!connected;
  const busy=!!S.operation||S.stopping>0;
  const source=p?.control?.source_id??2;
  // 切换开关只在输出停止、未在校准时可用；切换本身也会让固件先停止输出。
  $('use-imu2').checked=source===2;$('use-imu2').disabled=!(connected&&isFresh&&!busy&&!outputsActive()&&!p?.calibrating);
  $('active-source-name').textContent=source===2?'IMU2 · 0x69':'IMU1 · 0x68';
  $('active-source-path').textContent=source===2?'主控本地 I2C':'SuperMini → UART → 主控';
  $('source-state').textContent=!connected?'默认 IMU2；连接后以设备回传为准':busy&&S.operation?.kind==='source'?'正在切换，输出保持停止':p?.calibrating?`页面选择 IMU${source} · 固件生效 IMU${source} · 竖直校准中`:`页面选择 IMU${source} · 固件生效 IMU${source}（${source===2?'本地 I2C 0x69':'SuperMini 0x68'}）`;
  const mountYaw=p?.control?.mount_yaw_deg;
  $('mounting-state').textContent=!isFresh?'安装偏角等待设备回传':!Number.isFinite(mountYaw)?'当前固件未报告安装偏角；请上传 2.4.3 主控':`IMU${source} 安装偏角 ${mountYaw}° · ${p.control.mount_ready?'已纳入姿态与 PID':'等待竖直校准'}（俯视逆时针为正）`;
  $('imu2-role').textContent=source===2?'当前控制源；下列为本地原始校验数据':'备用监测；不参与当前 PID';
  $('tab-auto').disabled=$('tab-manual').disabled=busy;
  const auto=isFresh&&!!p?.control?.auto_enabled,world=isFresh&&!!p?.world?.valid;
  $('auto-state').textContent=p?.control?.preposition_active?'启动前舵机预定位（预计等待）':auto?(p?.outputs?.esc_armed?'正在回正':'舵机实时纠偏 · 电机停止'):p?.calibrating?'正在校准':'未启动';$('auto-state').classList.toggle('live',auto);
  $('rebuild-world').disabled=!(connected&&isFresh&&attitude&&!p.calibrating&&!outputsActive()&&!busy);
  $('auto-start').disabled=!(connected&&ok&&attitude&&world&&p.control?.mount_ready&&p.control?.tilt_valid&&!auto&&(!(p.outputs.esc_armed||p.outputs.esc_arming)||p.control?.auto_motor_allowed)&&!busy);
  $('auto-stop').disabled=!(auto||S.autoFanWanted||S.autoThrottleWanted||p?.outputs.esc_armed||p?.outputs.esc_arming||S.operation?.kind==='auto'||S.operation?.kind==='auto-motor');
  $('auto-start').textContent=S.autoThrottleWanted?'开始回中 + 自动油门':'开始姿态回中';
  const throttle=isFresh?p?.control?.tilt_throttle:null;
  $('tilt-throttle-enable').checked=S.autoThrottleWanted;
  $('tilt-throttle-enable').disabled=!(connected&&isFresh&&attitude&&(S.autoThrottleWanted||(ok&&world&&!busy&&!outputsActive()&&!S.autoFanWanted&&throttle)));
  for(const id of Object.keys(throttleInputs))$(id).disabled=busy||outputsActive();
  $('tilt-throttle-state').textContent=throttle?.enabled?'按倾斜自动调节':S.autoThrottleWanted?(p?.outputs.esc_arming?'电调启动准备中':'已允许 · 点击开始回中联动启动'):'关闭';
  $('tilt-throttle-angle').textContent=throttle?.enabled?`${throttle.tilt_deg.toFixed(1)}°`:'—°';
  $('tilt-throttle-target').textContent=throttle?.enabled?`${throttle.target_pct.toFixed(1)}%`:'—%';
  $('tilt-throttle-output').textContent=throttle?.enabled?`${throttle.output_pct.toFixed(1)}% / ${p.outputs.esc_us}µs`:'停止';
  $('tilt-throttle-stop').disabled=!(p?.outputs.esc_armed||p?.outputs.esc_arming||S.autoThrottleWanted);
  const error=isFresh&&p?.world?.valid&&p?.control?.tilt_valid&&p?.control?.error_deg||[NaN,NaN],euler=isFresh&&p?.world?.valid&&p?.world?.euler_deg||[NaN,NaN,NaN];
  $('pitch-value').textContent=Number.isFinite(error[1])?`${error[1].toFixed(1)}°`:'—°';$('roll-value').textContent=Number.isFinite(error[0])?`${error[0].toFixed(1)}°`:'—°';$('yaw-value').textContent=Number.isFinite(euler[2])?`${euler[2].toFixed(1)}°`:'—°';
  const sampleState=p?.control_imu?.history_state;
  $('imu-status').textContent=!p?'等待':!isFresh?'连接中断':p.calibrating?'校准中':p.control_imu?.valid?({PREDICTED:'历史估算',HELD:'历史保持'})[sampleState]||'实时':({WAIT_SAMPLE:'等待新样本',NOT_FOUND:'未检测到',WAIT_INT:'等待中断',STALE:'历史超时',READ_ERROR:'读取异常'})[p.control_imu?.status]||'异常';
  $('imu-status').classList.toggle('history-reading',isFresh&&['PREDICTED','HELD'].includes(sampleState));
  $('world-status').textContent=world?`已建立 #${p.world.reference_id}`:'未建立';$('sample-rate').textContent=isFresh&&Number.isFinite(p?.sample_hz)?`${p.sample_hz} Hz`:'— Hz';
  $('auto-servo-x').textContent=isFresh?`${usToAngle(p.outputs.servo_us[0],0)}°`:'—°';$('auto-servo-y').textContent=isFresh?`${usToAngle(p.outputs.servo_us[1],1)}°`:'—°';
  const servoOn=!!p?.outputs.servo_enabled&&!auto;
  $('servo-enable').disabled=!(connected&&ok&&manual&&!servoOn&&!busy);$('servo-disable').disabled=!(servoOn||S.operation?.kind==='servo');$('servo-x').disabled=!(manual&&ok&&servoOn);$('servo-y').disabled=!(manual&&ok&&servoOn);$('servo-center').disabled=!(manual&&ok&&servoOn);
  const armed=!!p?.outputs.esc_armed,arming=!!p?.outputs.esc_arming;$('motor-state').textContent=arming?'启动准备中':armed?`转动中 · ${Math.max(0,Math.round((p.outputs.esc_us-1500)/2))}%`:'已停止';$('motor-state').classList.toggle('live',armed||arming);
  $('motor-start').disabled=!(connected&&ok&&manual&&$('prop-confirm').checked&&!armed&&!arming&&!busy);$('motor-stop').disabled=!(armed||arming||p?.outputs.esc_signal_enabled||S.operation?.kind==='motor');
  const permitted=attitude&&!!p?.control?.auto_motor_allowed;
  $('auto-motor-enable').checked=S.autoFanWanted;
  $('auto-motor-enable').disabled=S.autoThrottleWanted||!!throttle?.enabled||!(connected&&isFresh&&attitude&&((ok&&world&&!busy)||S.autoFanWanted));
  $('auto-motor-state').textContent=p?.control?.preposition_active?'舵机预定位中 · 电机停止':arming?'电调启动准备中':armed?`已启用 · ${Math.max(0,Math.round((p.outputs.esc_us-1500)/2))}%`:S.autoFanWanted&&permitted?'已允许调速 · 已停止':'未启用';
  $('auto-motor-state').classList.toggle('live',attitude&&(armed||arming));
  $('auto-motor-start').disabled=S.autoThrottleWanted||!!throttle?.enabled||!(connected&&ok&&world&&permitted&&S.autoFanWanted&&!armed&&!arming&&!busy);
  $('auto-motor-stop').disabled=!(attitude&&(armed||arming||S.operation?.kind==='auto-motor'));
  $('auto-motor-speed').disabled=!!throttle?.enabled||!(connected&&ok&&permitted&&S.autoFanWanted&&armed&&!busy);
  updateLabels();renderImu1(p,isFresh);renderImu2(p,isFresh);renderDetails(p,isFresh,connected,busy);attitudePreview.render(p,isFresh);
}

$('connect-usb').addEventListener('click',connectUsb);$('connect-wifi').addEventListener('click',connectWifi);$('disconnect').addEventListener('click',()=>disconnect());$('stop-all').addEventListener('click',()=>run('舵机和电机已全部停止。',stopAll));
$('copy-fault').addEventListener('click',copyFault);
$('use-imu2').addEventListener('change',()=>{const source=$('use-imu2').checked?2:1;run(`已切换到 IMU${source}，输出已停止。请保持箭体竖直静止，校准后手动启动。`,()=>switchImuSource(source));});
$('tab-auto').addEventListener('click',()=>showPage('auto'));$('tab-manual').addEventListener('click',()=>showPage('manual'));
$('rebuild-world').addEventListener('click',()=>run('正在校准：箭体保持竖直、箭头朝上，PCB 板面竖直，约需3秒。',recalibrate));
$('auto-start').addEventListener('click',()=>run('自动回正已启动。',startAutomatic));
$('auto-stop').addEventListener('click',()=>run('自动控制已停止。',stopAutomatic));
$('tilt-throttle-enable').addEventListener('change',()=>run($('tilt-throttle-enable').checked?'已允许倾斜自动油门，点击开始回中后联动启动。':'自动油门已关闭，电机已停止。',toggleTiltThrottle));
$('tilt-throttle-stop').addEventListener('click',()=>run('电机已停止，舵机回中保持运行。',stopMotor));
$('servo-enable').addEventListener('click',()=>run('两个舵机已启用。',enableServos));$('servo-disable').addEventListener('click',()=>run('两个舵机已停止。',disableServos));
$('servo-center').addEventListener('click',()=>{$('servo-x').value=180;$('servo-y').value=135;updateLabels();run('已发送方位180° / 俯仰135°回中指令。',sendServo);});
$('pid-form').addEventListener('input',()=>{S.pidDirty=true;render();});
$('pid-form').addEventListener('submit',event=>{event.preventDefault();run('PID 与映射参数已由设备回传确认。',applyPid);});
window.addEventListener('resize',()=>drawTrend(fresh()));
for(const id of ['servo-x','servo-y'])$(id).addEventListener('input',()=>{updateLabels();queueServo();});
$('auto-motor-enable').addEventListener('change',()=>run($('auto-motor-enable').checked?'已允许自动模式下手动调速，风扇尚未启动。':'风扇已停止并关闭调速。',toggleAutoMotor));
$('auto-motor-start').addEventListener('click',()=>run('风扇已启用，可调节速度。',startAutoMotor));
$('auto-motor-stop').addEventListener('click',()=>run('风扇已停止，自动回正保持运行。',stopMotor));
$('auto-motor-speed').addEventListener('input',()=>{updateLabels();queueAutoMotor();});
$('motor-speed').addEventListener('input',()=>{updateLabels();queueMotor();});$('prop-confirm').addEventListener('change',render);$('motor-start').addEventListener('click',()=>run('电机已按当前速度启动。',startMotor));$('motor-stop').addEventListener('click',()=>run('电机已停止，舵机保持可控。',stopMotor));

setInterval(()=>{
  if(S.mode&&S.boot&&!document.hidden&&fresh()){
    const generation=S.generation;
    // 心跳按200ms独立发送；一帧ACK延迟或丢失不能阻塞后续心跳。
    if(S.heartbeat)command('heartbeat',{},800).catch(error=>{if(generation===S.generation)notice(`心跳回复异常：${error.message}`,'error');});
    else if(!S.healthPending&&performance.now()-S.lastKeepalive>=1000){
      S.healthPending=true;S.lastKeepalive=performance.now();
      command('keepalive').catch(error=>{if(generation===S.generation)notice(error.message,'error');}).finally(()=>{if(generation===S.generation)S.healthPending=false;});
    }
  }
  if(S.mode&&(S.heartbeat||S.operation)&&!fresh()){resetAutoMotor();cancelActions();S.heartbeat=false;void command('stop').catch(()=>{});notice('设备数据中断，已发送全部停止。','error');}
  render();
},200);

document.addEventListener('visibilitychange',()=>{
  if(document.hidden){resetAutoMotor();const active=S.heartbeat||outputsActive()||S.operation;cancelActions();S.heartbeat=false;if(S.mode&&active)void stopAll().catch(()=>{});}
});
window.addEventListener('pagehide',()=>{resetAutoMotor();cancelActions();S.heartbeat=false;if(S.mode&&S.boot)void command('stop',{},300).catch(()=>{});});
updateLabels();render();
if(location.hostname==='192.168.4.1')setTimeout(connectWifi,150);

})();
