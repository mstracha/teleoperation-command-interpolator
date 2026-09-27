import fs from "node:fs";
import path from "node:path";
import { pathToFileURL, fileURLToPath } from "node:url";

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..");
const htmlPath = path.join(root, "exports", "TeleoperationCommandInterpolatorDesign.html");
const pdfPath = path.join(root, "exports", "TeleoperationCommandInterpolatorDesign.pdf");
const chromePath = "C:\\Program Files\\Google\\Chrome\\Application\\chrome.exe";

let puppeteer;

try {
  ({ default: puppeteer } = await import("puppeteer"));
} catch {
  const pnpmDirectory = path.join(root, "node_modules", ".pnpm");
  const packageDirectory = fs.readdirSync(pnpmDirectory)
    .find((name) => name.startsWith("puppeteer@"));

  if (!packageDirectory) {
    throw new Error("The Puppeteer package is not installed.");
  }

  const fallback = pathToFileURL(path.join(
    pnpmDirectory,
    packageDirectory,
    "node_modules",
    "puppeteer",
    "lib",
    "puppeteer",
    "puppeteer.js"
  )).href;
  ({ default: puppeteer } = await import(fallback));
}

if (!fs.existsSync(htmlPath)) {
  throw new Error("Build the HTML export before generating the PDF.");
}

if (!fs.existsSync(chromePath)) {
  throw new Error(`Chrome was not found at ${chromePath}`);
}

const browser = await puppeteer.launch({
  executablePath: chromePath,
  headless: true,
  args: [
    "--no-sandbox",
    "--disable-gpu",
    "--disable-software-rasterizer",
    "--disable-gpu-sandbox",
    "--disable-dev-shm-usage"
  ]
});

try {
  const page = await browser.newPage();
  await page.goto(pathToFileURL(htmlPath).href, { waitUntil: "networkidle0" });
  await page.pdf({
    path: pdfPath,
    format: "A4",
    printBackground: true,
    displayHeaderFooter: false,
    preferCSSPageSize: true
  });
} finally {
  await browser.close();
}

console.log(pdfPath);
