import fs from "node:fs";
import path from "node:path";
import { spawnSync } from "node:child_process";
import { fileURLToPath } from "node:url";

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..");
const sourceDirectory = path.join(root, "diagrams", "src");
const outputDirectory = path.join(root, "diagrams", "rendered");
const configurationPath = path.join(root, "tools", "puppeteer-config.json");
const cliPath = path.join(
  root,
  "node_modules",
  "@mermaid-js",
  "mermaid-cli",
  "src",
  "cli.js"
);

fs.mkdirSync(outputDirectory, { recursive: true });

const sources = fs.readdirSync(sourceDirectory)
  .filter((name) => name.endsWith(".mmd"))
  .sort();

for (const sourceName of sources) {
  const inputPath = path.join(sourceDirectory, sourceName);
  const stem = path.basename(sourceName, ".mmd");

  for (const extension of ["svg", "png"]) {
    const outputPath = path.join(outputDirectory, `${stem}.${extension}`);
    const result = spawnSync(process.execPath, [
      cliPath,
      "-p", configurationPath,
      "-i", inputPath,
      "-o", outputPath,
      "-b", "transparent",
      "-s", "2"
    ], { stdio: "inherit" });

    if (result.status !== 0) {
      throw new Error(`Mermaid rendering failed for ${sourceName} (${extension})`);
    }
  }
}

console.log(`Rendered ${sources.length} diagrams as SVG and PNG in ${outputDirectory}`);
