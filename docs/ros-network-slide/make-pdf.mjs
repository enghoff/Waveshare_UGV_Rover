// Turn the canvas artboard into a standalone page a browser can print, then
// print it. Two steps, because the PDF has to come out of a real browser:
//
//     node make-pdf.mjs
//     chrome --headless=new --disable-gpu --no-pdf-header-footer \
//            --virtual-time-budget=10000 \
//            --print-to-pdf=rover-ros2-network.pdf print.html
//
// Chrome's own printer is what keeps the result vector: paths stay paths and
// text stays text in embedded font subsets. The canvas page's Export PDF
// button rasterises instead, which is why the committed file is made here.
//
// Chrome returns before it has finished writing, so wait for the file rather
// than for the process.
import { readFileSync, writeFileSync } from "node:fs";

let html = readFileSync("Main.dc.html", "utf8");

// The two hooks the canvas runtime owns, and nothing else: the artboard's own
// markup has to reach the printer exactly as the canvas renders it.
html = html.replace(/\s*<script src="\.\/support\.js"><\/script>/, "");
html = html.replace(/\s*<script data-dc-script[\s\S]*?<\/script>/, "");

// A page box the size of the slide, so the print is one page at natural size
// rather than a letter page with the slide scaled onto it.
html = html.replace(
  "<style>",
  `<style>
    @page { size: 13.333in 7.5in; margin: 0; }
    html, body { margin: 0; padding: 0; }
    x-dc { display: block; }
    * { -webkit-print-color-adjust: exact; print-color-adjust: exact; }`
);

writeFileSync("print.html", html);
console.log("print.html written, " + html.length + " bytes");
