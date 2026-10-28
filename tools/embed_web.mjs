// 固件网页资源生成步骤，以 web/dist 中的文件为源文件。
import {readFileSync,writeFileSync} from 'node:fs';
import {fileURLToPath} from 'node:url';
import {dirname,resolve} from 'node:path';
import {gzipSync,gunzipSync} from 'node:zlib';
import {createHash} from 'node:crypto';
const root=resolve(dirname(fileURLToPath(import.meta.url)),'..');
// 生成普通脚本，使本地 file:// 页面也能运行，不受模块跨域加载限制。
// 隔离协议模块的作用域，避免与网页内部变量重名。
const dist=resolve(root,'web/dist');
const app=readFileSync(resolve(dist,'app.mjs'),'utf8');
let prelude='';
const appBody=app.replace(/^import \{([^}]+)\} from '\.\/(protocol|preview)\.mjs';\r?\n/gm,(_,bindings,file)=>{
  const names=bindings.split(',').map(s=>s.trim());
  if(names.some(s=>!/^[$A-Z_a-z][$\w]*$/.test(s)))throw new Error('Unsupported import binding');
  const body=readFileSync(resolve(dist,file+'.mjs'),'utf8').replace(/^export (?=const |function |class )/gm,'');
  if(/^\s*(import|export)\b/m.test(body))throw new Error('Unsupported module syntax');
  prelude+=`const {${names.join(',')}}=(()=>{\n${body}\nreturn {${names.join(',')}};})();\n`;
  return '';
});
if(/^\s*import\b/m.test(appBody))throw new Error('Unsupported app import');
const bundle=`// 由 tools/embed_web.mjs 自动生成，请勿手动修改。\n(function(){'use strict';\n${prelude}\n${appBody}\n})();\n`;
writeFileSync(resolve(dist,'app.bundle.js'),bundle,'utf8');
// 生成包含全部样式和脚本的单文件入口，复制后也不依赖旁边的其他文件。
const html=readFileSync(resolve(dist,'index.html'),'utf8');
const css=readFileSync(resolve(dist,'style.css'),'utf8');
const local=html.replace(/<link rel="stylesheet" href="style\.css[^" ]*">/,()=>`<style>${css}</style>`)
  .replace(/<script defer src="app\.bundle\.js[^" ]*"><\/script>/,()=>`<script>${bundle.replace(/<\/script/gi,'<\\/script')}</script>`);
if(local===html||local.includes('<script defer src='))throw new Error('Standalone build failed');
writeFileSync(resolve(root,'../TVC控制台.html'),local,'utf8');
const assets=[['index.html','WEB_INDEX_GZ'],['app.mjs','WEB_APP_GZ'],['app.bundle.js','WEB_BUNDLE_GZ'],['protocol.mjs','WEB_PROTOCOL_GZ'],['preview.mjs','WEB_PREVIEW_GZ'],['style.css','WEB_STYLE_GZ']];
let header='// 由 tools/embed_web.mjs 根据 web/dist 自动生成，请勿手动修改。\n#pragma once\n#include <Arduino.h>\n\n';
let total=0;
for(const [file,name] of assets){
  const original=readFileSync(resolve(root,'web/dist',file)),packed=gzipSync(original,{level:9});
  if(!gunzipSync(packed).equals(original))throw new Error('Asset roundtrip failed: '+file);
  const hash=createHash('sha256').update(original).digest('hex');
  header+='// '+file+' SHA256 '+hash+'\nstatic const uint8_t '+name+'[] PROGMEM = {\n';
  for(let offset=0;offset<packed.length;offset+=20)header+='  '+Array.from(packed.subarray(offset,offset+20),b=>'0x'+b.toString(16).padStart(2,'0')).join(',')+',\n';
  header+='};\n\n';total+=packed.length;
  console.log(file+': '+original.length+' -> '+packed.length+' bytes');
}
const target=resolve(root,'WebAssets.h');
writeFileSync(target,header,'utf8');
console.log('Embedded '+total+' gzip bytes in '+target);
