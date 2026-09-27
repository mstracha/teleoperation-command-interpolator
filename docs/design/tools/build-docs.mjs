import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { marked } from "marked";

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..");
const sourcePath = path.join(root, "TeleoperationCommandInterpolatorDesign.md");
const outputDirectory = path.join(root, "exports");
const outputPath = path.join(outputDirectory, "TeleoperationCommandInterpolatorDesign.html");

fs.mkdirSync(outputDirectory, { recursive: true });

const markdown = fs.readFileSync(sourcePath, "utf8");
const sectionTitles = [...markdown.matchAll(/^##\s+(.+)$/gm)].map((match) => match[1]);
let rendered = marked.parse(markdown, { gfm: true });
rendered = rendered.replaceAll('src="diagrams/', 'src="../diagrams/');

function keepTogether(html, startMarker, endMarker) {
  const start = html.indexOf(startMarker);
  const end = html.indexOf(endMarker, start + startMarker.length);

  if (start < 0 || end < 0) {
    throw new Error(`Unable to locate keep-together range: ${startMarker}`);
  }

  return `${html.slice(0, start)}<section class="keep-together">${html.slice(start, end)}</section>\n${html.slice(end)}`;
}

rendered = keepTogether(
  rendered,
  '<h4><code>InterpolatorStateMachine</code></h4>',
  '<h4><code>ShutdownCoordinator</code></h4>',
);
rendered = keepTogether(
  rendered,
  '<h4><code>RunReportWriter</code></h4>',
  '<h2>6. Data model and result taxonomy</h2>',
);
rendered = keepTogether(
  rendered,
  '<h2>13. Improvement loop</h2>',
  '<h2>14. Readiness assessment</h2>',
);

const firstSection = rendered.indexOf("<h2>");
const preamble = firstSection >= 0 ? rendered.slice(0, firstSection) : rendered;
const sections = firstSection >= 0 ? rendered.slice(firstSection) : "";

const contents = sectionTitles
  .map((title) => `<li>${escapeHtml(title)}</li>`)
  .join("\n");

const html = `<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Teleoperation Command Interpolator - Design Record</title>
<style>
  :root { --ink:#17212b; --muted:#536270; --blue:#345b78; --deep:#173f5b; --pale:#eaf1f8; --rule:#c8d2da; --warn:#b04444; }
  * { box-sizing:border-box; }
  html { background:#edf1f4; }
  body { max-width:1040px; margin:24px auto; padding:54px 70px; background:#fff; color:var(--ink); font:16px/1.55 Arial, Helvetica, sans-serif; box-shadow:0 2px 18px #0002; }
  .cover { min-height:88vh; display:flex; flex-direction:column; justify-content:center; }
  .cover h1 { font-size:2.55rem; }
  h1 { color:var(--deep); font-size:2.35rem; line-height:1.1; margin:0 0 .55rem; border-bottom:4px solid var(--blue); padding-bottom:.5rem; }
  h2 { color:var(--deep); font-size:1.65rem; line-height:1.2; margin:2.5rem 0 .8rem; border-bottom:1px solid var(--rule); padding-bottom:.25rem; break-after:avoid-page; page-break-after:avoid; }
  h3 { color:#28546f; font-size:1.25rem; margin:1.8rem 0 .55rem; break-after:avoid-page; page-break-after:avoid; }
  h4 { font-size:1.05rem; margin:1.25rem 0 .35rem; break-after:avoid-page; page-break-after:avoid; }
  h2 + *, h3 + *, h4 + * { break-before:avoid-page; page-break-before:avoid; }
  .keep-together { break-inside:avoid-page; page-break-inside:avoid; }
  p, li { orphans:3; widows:3; }
  blockquote { margin:1.1rem 0; padding:.8rem 1rem; border-left:5px solid var(--warn); background:#fff4f2; color:#4a2525; }
  blockquote strong:first-child { color:#7d2929; }
  code { font-family:Consolas, "Courier New", monospace; background:#f1f4f6; padding:.08rem .25rem; border-radius:3px; }
  pre { background:#17212b; color:#f4f7fa; padding:1rem; overflow:auto; border-radius:5px; break-inside:avoid; font-size:.86rem; }
  pre code { background:transparent; color:inherit; padding:0; }
  table { width:100%; border-collapse:collapse; margin:1rem 0 1.4rem; font-size:.91rem; break-inside:avoid; }
  th { background:var(--pale); color:var(--deep); text-align:left; }
  th, td { border:1px solid #aebbc5; padding:.45rem .55rem; vertical-align:top; }
  tr { break-inside:avoid; }
  img { display:block; max-width:100%; max-height:840px; width:auto; height:auto; margin:1rem auto 1.5rem; break-inside:avoid; }
  hr { border:0; border-top:1px solid var(--rule); margin:2rem 0; }
  a { color:#255c80; }
  .contents { margin:2rem 0; padding:1.1rem 1.4rem; background:#f5f8fa; border:1px solid var(--rule); border-radius:6px; }
  .contents h2 { margin:0 0 .6rem; border:0; font-size:1.2rem; }
  .contents ul { columns:2; column-gap:2rem; margin:.4rem 0; padding-left:1.4rem; font-size:.9rem; }
  .contents li { break-inside:avoid; margin:.2rem 0; }
  .page-break { break-before:page; }
  @media print {
    @page { size:A4; margin:14mm 15mm 16mm; }
    html, body { background:#fff; }
    body { max-width:none; margin:0; padding:0; box-shadow:none; font-size:9.2pt; line-height:1.38; }
    .cover { min-height:250mm; break-after:page; }
    .cover h1 { font-size:25pt; }
    h1 { font-size:23pt; }
    h2 { font-size:15.5pt; margin-top:1.55rem; }
    h3 { font-size:12pt; margin-top:1.15rem; }
    h4 { font-size:10.2pt; }
    table { font-size:7.8pt; }
    pre { font-size:7.5pt; white-space:pre-wrap; overflow-wrap:anywhere; }
    img { max-height:238mm; }
    .contents { break-after:page; }
    .page-break { break-before:page; }
    a { color:inherit; text-decoration:none; }
  }
</style>
</head>
<body>
<section class="cover">${preamble}</section>
<aside class="contents"><h2>Contents</h2><ul>${contents}</ul></aside>
${sections}
</body>
</html>`;

fs.writeFileSync(outputPath, html, "utf8");
console.log(outputPath);

function escapeHtml(value) {
  return value
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;");
}
