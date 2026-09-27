# Teleoperation Command Interpolator design documentation

This directory contains the human-readable and agent-readable design record for the teleoperation command interpolator.

- `TeleoperationCommandInterpolatorDesign.md` is the canonical source document.
- `diagrams/src/` contains editable Mermaid sources.
- `diagrams/rendered/` contains rendered SVG and PNG figures.
- `exports/TeleoperationCommandInterpolatorDesign.html` is the browser edition.
- `exports/TeleoperationCommandInterpolatorDesign.pdf` is the printable edition.

The Markdown and Mermaid files are the maintainable source of truth. Generated HTML, PDF, and SVG files should be regenerated after material source changes.

## Rebuild

With Node.js and Google Chrome installed:

```powershell
npm install
npm run diagrams
npm run html
npm run pdf
```

Visually inspect every PDF page after regeneration.

This project is a teaching and test system. Its fictional joint limits and process-local harnesses are not evidence that it is safe to command physical hardware.
