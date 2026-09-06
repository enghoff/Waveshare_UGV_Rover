// Turn the canvas artboards into one standalone page a browser can print, then
// print it. Two steps, because the PDF has to come out of a real browser:
//
//     node make-pdf.mjs
//     chrome --headless=new --disable-gpu --no-pdf-header-footer \
//            --virtual-time-budget=10000 \
//            --print-to-pdf=rover-architecture.pdf print.html
//
// Chrome's own printer is what keeps the result vector: paths stay paths and
// text stays text in embedded font subsets. The canvas page's Export PDF
// button rasterises instead, which is why the committed file is made here.
//
// Chrome returns before it has finished writing, so wait for the file rather
// than for the process.
//
// The page order is canvas.json's artboard order, so the document reads the way
// the canvas is laid out and neither has to be kept in step with a list here.
import { readFileSync, writeFileSync } from "node:fs";

const canvas = JSON.parse(readFileSync("canvas.json", "utf8"));

// Each artboard is a whole HTML document whose <x-dc> element is the slide. Only
// that element is taken: the two hooks around it -- the canvas runtime's
// support.js and its logic block -- belong to the editor and not to a printer.
// The <helmet> inside carries the font link and the page's own styles, and
// repeating those per slide is harmless because every artboard's are identical.
const pages = canvas.artboards.map(({ file }) => {
  const html = readFileSync(file, "utf8");
  const slide = html.match(/<x-dc>[\s\S]*?<\/x-dc>/);
  if (!slide) throw new Error(`${file} has no <x-dc> element`);
  return `<div class="dc-page">\n${slide[0]}\n</div>`;
});

// A page box the size of a slide, so each print is one page at natural size
// rather than a letter page with the slide scaled onto it. The last slide must
// not break after it, or Chrome emits a blank final page.
const out = `<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <style>
    @page { size: 13.333in 7.5in; margin: 0; }
    html, body { margin: 0; padding: 0; }
    x-dc { display: block; }
    .dc-page { break-after: page; }
    .dc-page:last-child { break-after: auto; }
    * { -webkit-print-color-adjust: exact; print-color-adjust: exact; }
  </style>
</head>
<body>
${pages.join("\n")}
</body>
</html>
`;

writeFileSync("print.html", out);
console.log(`print.html written, ${canvas.artboards.length} pages, ${out.length} bytes`);
