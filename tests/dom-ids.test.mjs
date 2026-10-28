import {readFileSync} from 'node:fs';
import {fileURLToPath} from 'node:url';
import {dirname,resolve} from 'node:path';
const root=resolve(dirname(fileURLToPath(import.meta.url)),'..');
const html=readFileSync(resolve(root,'web/dist/index.html'),'utf8');
const script=readFileSync(resolve(root,'web/dist/app.mjs'),'utf8')+'\n'+readFileSync(resolve(root,'web/dist/preview.mjs'),'utf8');
const references=[...script.matchAll(/(?:\$|get)\('([^']+)'\)/g)].map(match=>match[1]);
const declared=[...html.matchAll(/id="([^"]+)"/g)].map(match=>match[1]);
const missing=[...new Set(references)].filter(id=>!declared.includes(id));
const duplicates=[...new Set(declared.filter((id,index)=>declared.indexOf(id)!==index))];
if(missing.length||duplicates.length){
  console.error({missing,duplicates});
  process.exit(1);
}
console.log(`PASS DOM ids: ${new Set(references).size} referenced, ${declared.length} declared`);
