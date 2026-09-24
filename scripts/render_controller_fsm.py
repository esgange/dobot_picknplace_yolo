#!/usr/bin/env python3
"""Export the FSM Markdown diagrams to an offline HTML viewer and a vector PDF.

Requires locally installed Playwright/Chromium and the VS Code Mermaid Markdown
preview bundle passed explicitly with --renderer. No network access is used.
"""

import argparse
import hashlib
import html
from pathlib import Path
import re

from playwright.sync_api import sync_playwright


STYLE = """
*{box-sizing:border-box}body{margin:0;font-family:Arial,sans-serif;color:#172c43;
background:#edf1f5}button,a{font:inherit}button{cursor:pointer}
header{padding:22px 28px;background:#10283f;color:white;display:flex;
align-items:center;justify-content:space-between;gap:20px}
.eyebrow{font-size:11px;letter-spacing:2px;color:#a8cbe4;text-transform:uppercase}
h1{font-size:26px;margin:7px 0 0;font-weight:600}header a{color:#d8eaf7;
font-size:13px;text-decoration:none;border:1px solid #456077;padding:10px 14px;
border-radius:7px}.links{display:flex;gap:10px}
.workspace{display:grid;grid-template-columns:260px minmax(0,1fr);gap:20px;
height:calc(100vh - 102px);min-height:480px;padding:20px}
aside{display:flex;flex-direction:column;gap:7px;padding:12px 4px}
.nav-label{font-size:11px;letter-spacing:1.5px;color:#62788b;padding:0 12px 10px}
nav{display:flex;flex-direction:column;gap:8px}nav button{text-align:left;
border:0;background:transparent;color:#486075;padding:14px 12px;border-radius:8px;
line-height:1.4;display:flex;gap:13px;align-items:baseline}
nav button span{font-size:12px;color:#7890a3}nav button[aria-pressed=true]{
background:#dbe9f4;color:#133f62;font-weight:600;box-shadow:inset 3px 0 #2375af}
nav button:hover{background:#e0e9f1}.help{font-size:12px;line-height:1.6;
color:#62788b;padding:14px 12px;margin-top:auto}
.viewer{min-width:0;min-height:0;display:flex;flex-direction:column;
background:white;border:1px solid #d7e1e9;border-radius:12px;overflow:hidden}
.toolbar{display:flex;align-items:center;justify-content:space-between;
gap:12px;border-bottom:1px solid #e2e9ef;padding:15px 18px}
h2{font-size:18px;margin:0}.zoom{display:flex;align-items:center;gap:6px}
.zoom button{background:#f4f7fa;border:1px solid #d5e0e9;border-radius:6px;
padding:8px 11px;color:#26465f}.zoom output{font-size:12px;min-width:48px;
text-align:center}.viewport{overflow:auto;flex:1;min-height:0;padding:24px;
cursor:grab;touch-action:pan-x pan-y}.viewport.dragging{cursor:grabbing;
user-select:none}.diagram{margin:0 auto}.diagram svg{display:block;max-width:none!important}
.print-title,.page-note{display:none}.footer{font-size:11px;color:#6f8190;
padding:11px 18px;border-top:1px solid #e2e9ef;line-height:1.5}
@media(max-width:850px){.workspace{grid-template-columns:1fr;height:auto}
aside{padding:0}nav{flex-direction:row;flex-wrap:wrap;gap:4px}
nav button{padding:8px;font-size:12px}.nav-label,.help{display:none}
.viewer{height:78vh;min-height:400px}header{padding:16px}h1{font-size:20px}
.toolbar{flex-wrap:wrap}.links a{font-size:12px;padding:8px}}
@page{size:A3 portrait;margin:15mm}
@media print{body{background:white;color:#172c43}header,aside,.toolbar,.footer{
display:none}.workspace,.viewer,.viewport{display:block;height:auto;min-height:0;
padding:0;margin:0;border:0;overflow:visible}.workspace{display:block}
article,article[hidden]{display:block!important;break-after:page}
article:last-child{break-after:auto}.print-title{display:block;font-size:22pt;
margin:0 0 7mm}.page-note{display:block;font-size:9pt;color:#526b7d;margin:5mm 0 0}
.diagram{width:267mm!important;height:350mm!important;margin:0}
.diagram svg{width:100%!important;height:100%!important;max-width:none!important}}
"""

INTERACTION = """
const articles = [...document.querySelectorAll('article')];
const buttons = [...document.querySelectorAll('nav button')];
const viewport = document.querySelector('.viewport');
const output = document.querySelector('output');
let current = 0, scale = 1;
function zoom(value) {
  scale = Math.max(0.2, Math.min(3, value));
  const diagram = articles[current].querySelector('.diagram');
  const svg = diagram.querySelector('svg');
  const box = svg.viewBox.baseVal;
  diagram.style.width = (box.width * scale) + 'px';
  diagram.style.height = (box.height * scale) + 'px';
  svg.style.width = '100%'; svg.style.height = '100%';
  output.value = Math.round(scale * 100) + '%';
}
function fit() {
  const box = articles[current].querySelector('svg').viewBox.baseVal;
  zoom(Math.min(1, (viewport.clientWidth - 48) / box.width));
}
function select(index) {
  current = index;
  articles.forEach((article, i) => { article.hidden = i !== index; });
  buttons.forEach((button, i) => button.setAttribute('aria-pressed', i === index));
  document.querySelector('#current-title').textContent = buttons[index].dataset.title;
  fit(); viewport.scrollTo(0, 0);
}
buttons.forEach((button, i) => button.addEventListener('click', () => select(i)));
document.querySelector('#zoom-in').onclick = () => zoom(scale * 1.25);
document.querySelector('#zoom-out').onclick = () => zoom(scale / 1.25);
document.querySelector('#actual').onclick = () => zoom(1);
document.querySelector('#fit').onclick = fit;
let drag;
viewport.addEventListener('pointerdown', event => {
  if (event.button !== 0 || event.pointerType !== 'mouse') return;
  drag = {x:event.clientX, y:event.clientY, left:viewport.scrollLeft,
          top:viewport.scrollTop};
  viewport.setPointerCapture(event.pointerId);
  viewport.classList.add('dragging');
});
viewport.addEventListener('pointermove', event => {
  if (!drag) return;
  viewport.scrollLeft = drag.left - event.clientX + drag.x;
  viewport.scrollTop = drag.top - event.clientY + drag.y;
});
function endDrag() { drag = null; viewport.classList.remove('dragging'); }
viewport.addEventListener('pointerup', endDrag);
viewport.addEventListener('pointercancel', endDrag);
window.addEventListener('resize', fit);
select(0);
"""


def diagrams(markdown):
    """Copy each Mermaid block verbatim and use its nearest document heading."""
    found = []
    heading = None
    block = None
    for line in markdown.splitlines():
        if block is not None:
            if line == '```':
                found.append((heading, '\n'.join(block)))
                block = None
            else:
                block.append(line)
        elif line == '```mermaid':
            if heading is None:
                raise ValueError('Mermaid diagram has no document heading')
            block = []
        elif line.startswith('#'):
            heading = re.sub(r'^\d+\.\s*', '', line.lstrip('#').strip())
    if block is not None or not found:
        raise ValueError('FSM document has no diagrams or an unclosed Mermaid block')
    return found


def viewer(charts, svgs, digest):
    navigation, articles = [], []
    for i, ((title, _source), svg) in enumerate(zip(charts, svgs), 1):
        name = html.escape(title)
        navigation.append(
            f'<button data-title="{name}" aria-pressed="false">'
            f'<span>{i:02}</span>{name}</button>')
        articles.append(
            f'<article aria-label="{name}"><h2 class="print-title">'
            f'{i:02} / {name}</h2><div class="diagram">{svg}</div>'
            f'<p class="page-note">PicknPlace / Controller FSM — {i} of {len(charts)}'
            f' · Source: ROBOT_CONTROLLER_FSM.md · SHA-256 {digest[:12]}</p></article>')
    return f'''<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="fsm-source-sha256" content="{digest}">
<title>PicknPlace — Controller FSM</title><style>{STYLE}</style></head>
<body><header><div><div class="eyebrow">PicknPlace / System guide</div>
<h1>Controller state machine</h1></div><div class="links">
<a href="ROBOT_CONTROLLER_FSM.md">Source document</a>
<a href="ROBOT_CONTROLLER_FSM.pdf">Open PDF</a></div></header>
<div class="workspace"><aside><div class="nav-label">{len(charts)} diagram views</div>
<nav aria-label="FSM diagrams">{''.join(navigation)}</nav>
<div class="help">Select a view, then zoom or drag to explore.<br>
The source document contains the full state and transition tables.</div></aside>
<main class="viewer"><div class="toolbar"><h2 id="current-title"></h2>
<div class="zoom" aria-label="Diagram zoom">
<button id="zoom-out" aria-label="Zoom out">−</button><output aria-live="polite"></output>
<button id="zoom-in" aria-label="Zoom in">+</button>
<button id="actual">100%</button><button id="fit">Fit width</button></div></div>
<div class="viewport">{''.join(articles)}</div>
<div class="footer">Rendered from ROBOT_CONTROLLER_FSM.md. Open the source for
conditions and exceptions. This is a documentation view, not live robot status.</div>
</main></div><script>{INTERACTION}</script></body></html>
'''


def export(renderer):
    root = Path(__file__).resolve().parents[1]
    source = root / 'docs/ROBOT_CONTROLLER_FSM.md'
    raw = source.read_bytes()
    charts = diagrams(raw.decode('utf-8'))
    digest = hashlib.sha256(raw).hexdigest()
    errors = []
    with sync_playwright() as playwright:
        browser = playwright.chromium.launch(headless=True)
        page = browser.new_page(viewport={'width': 1440, 'height': 1000})
        page.route('**/*', lambda route: route.abort())
        page.on('pageerror', lambda error: errors.append(str(error)))
        markup = ''.join(f'<div class="mermaid">{html.escape(code)}</div>'
                         for _title, code in charts)
        page.set_content('<html><body>' + markup + '</body></html>')
        page.add_script_tag(path=str(renderer.resolve(strict=True)))
        page.wait_for_function(
            'count => document.querySelectorAll(".mermaid svg").length === count',
            arg=len(charts), timeout=30000)
        if errors or page.locator('.error-icon').count():
            raise ValueError('Mermaid rendering failed: ' + '; '.join(errors))
        svgs = page.locator('.mermaid > svg').evaluate_all(
            '(elements) => elements.map(svg => svg.outerHTML)')
        if len(svgs) != len(charts):
            raise ValueError('Expected exactly one rendered SVG per source diagram')
        rendered = viewer(charts, svgs, digest)
        # The exported viewer has no renderer globals or extension event hooks.
        page = browser.new_page(viewport={'width': 1440, 'height': 1000})
        page.route('**/*', lambda route: route.abort())
        page.on('pageerror', lambda error: errors.append(str(error)))
        page.set_content(rendered)
        pdf = page.pdf(prefer_css_page_size=True, print_background=True)
        if errors:
            raise ValueError('Visual viewer failed: ' + '; '.join(errors))
        browser.close()
    source.with_suffix('.html').write_text(rendered, encoding='utf-8')
    source.with_suffix('.pdf').write_bytes(pdf)
    print(f'Exported {len(charts)} diagrams to {source.with_suffix(".html")} and .pdf')
    print(f'Source SHA-256: {digest}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--renderer', required=True, type=Path,
                        help='Local VS Code Mermaid markdown-preview-out/index.js bundle')
    export(parser.parse_args().renderer)


if __name__ == '__main__':
    main()
