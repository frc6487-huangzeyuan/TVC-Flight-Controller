import assert from 'node:assert/strict';
import {LineDecoder,encodeFrame,MAX_FRAME} from '../web/dist/protocol.mjs';
const packet={v:7,seq:1,type:'ack',session:'test-session',ok:true};
const text='#IMU1 C3_TX q=100 s=100 st=2 a=0,0,8192 g=0,0,0 t=0 age=6000 r=TX\r\n'+encodeFrame(packet)+
  '#IMU1 S3_RX q=100 s=100 st=2 a=0,0,8192 g=0,0,0 t=0 age=6000 r=OK\n';
for(const chunk of [1,3,50,text.length]){
  const decoder=new LineDecoder(),frames=[],errors=[];
  for(let i=0;i<text.length;i+=chunk)decoder.push(text.slice(i,i+chunk),p=>frames.push(p),e=>errors.push(e));
  assert.deepEqual(frames,[packet]);assert.equal(errors.length,0);
}
const d=new LineDecoder(),errors=[],frames=[];
d.push('#OTHER invalid\n#IMU1 '+'x'.repeat(MAX_FRAME)+'\n'+encodeFrame(packet),p=>frames.push(p),e=>errors.push(e));
assert.equal(errors.length,2);assert.deepEqual(frames,[packet]);
console.log('PASS debug lines coexist with chunked protocol packets; corruption and oversized lines remain checked.');
