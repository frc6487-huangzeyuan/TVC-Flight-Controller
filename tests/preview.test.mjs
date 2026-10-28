import assert from 'node:assert/strict';
import {axisMapping,determinant3,eulerMatrix,transform3,transpose3,multiply3,remapAttitude,gimbalDirection,staticGimbal,sceneSvg} from '../web/dist/preview.mjs';
let checks=0;
const near=(a,b)=>{assert.equal(a.length,b.length);a.forEach((v,i)=>assert(Math.abs(v-b[i])<1e-9,`${v} != ${b[i]}`));checks++;};
const identity=[1,0,0,0,1,0,0,0,1];
near(transform3(eulerMatrix(0,0,90),[1,0,0]),[0,1,0]);
near(transform3(eulerMatrix(0,90,0),[0,0,1]),[1,0,0]);
near(transform3(eulerMatrix(90,0,0),[0,1,0]),[0,0,1]);
const permutations=[[1,2,3],[1,3,2],[2,1,3],[2,3,1],[3,1,2],[3,2,1]];
let right=0,left=0;
for(const p of permutations)for(let mask=0;mask<8;mask++){
  const axes=p.map((n,k)=>mask&(1<<k)?-n:n),m=axisMapping(axes);
  near(multiply3(m,transpose3(m)),identity);
  near(transform3(m,[2,3,5]),axes.map(n=>Math.sign(n)*[2,3,5][Math.abs(n)-1]));
  determinant3(m)>0?right++:left++;
  const original=eulerMatrix(22,-31,47),mount=eulerMatrix(13,0,78);
  near(remapAttitude(original,mount,mount),original);
  near(multiply3(remapAttitude(original,mount,m),m),multiply3(original,mount));
}
assert.equal(right,24);assert.equal(left,24);
for(const axes of [[1,1,3],[1,2,0],[1,2,4],[1,2],[1,2,NaN]])assert.throws(()=>axisMapping(axes));
for(const az of [0,90,180,270,360])for(const el of [105,120,135,150,165]){
  const d=gimbalDirection(az,el);assert(Math.abs(Math.hypot(...d)-1)<1e-12);checks++;
}
near(gimbalDirection(180,150),[Math.sin(Math.PI/12),0,Math.cos(Math.PI/12)]);
near(gimbalDirection(270,150),[0,Math.sin(Math.PI/12),Math.cos(Math.PI/12)]);
near(gimbalDirection(180,120),[-Math.sin(Math.PI/12),0,Math.cos(Math.PI/12)]);
const cfg={kp_x:.6,kp_y:.6,sign_x:1,sign_y:1,max_tilt_deg:15,azimuth_zero_deg:180};
let result=staticGimbal(eulerMatrix(0,10,0),cfg);
assert.equal(result.azimuth,180);assert(Math.abs(result.elevation-129)<1e-9);
result=staticGimbal(eulerMatrix(0,60,0),cfg);assert.equal(result.elevation,120);
result=staticGimbal(identity,cfg,287);assert.equal(result.azimuth,287);assert.equal(result.elevation,135);
result=staticGimbal(eulerMatrix(0,10,0),{...cfg,sign_x:-1});assert(Math.abs(result.elevation-141)<1e-9);
const svg=sceneSvg(eulerMatrix(25,-10,45),210,150);
assert(!/NaN|Infinity|undefined/.test(svg));
for(const text of ['B+X','B−X','B+Y','B−Y','B+Z','B−Z','S+X','W+Z','推力','喷口'])assert(svg.includes(text));
assert(sceneSvg(identity,180,135,{valid:false}).includes('等待有效姿态'));
assert(sceneSvg(identity,180,135,{mirrored:true}).includes('镜像坐标预览'));
console.log(`PASS ${checks} preview geometry checks, 24 right-handed / 24 mirrored mappings, TVC quadrants and invalid-axis rejection.`);
