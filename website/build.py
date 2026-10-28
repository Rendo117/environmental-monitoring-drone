#!/usr/bin/env python3
"""
build.py — static site generator for the project website (GitHub Pages, folder /docs).

  * pages/*.html  : hand-written page bodies
  * ../documentation/*.md : rendered to HTML pages (single source of truth with the repo docs)
  * math between $$…$$ (display) or $…$ (inline) is typeset with LaTeX and embedded as SVG,
    so the published site needs no JavaScript library and makes no third-party requests.

Requirements: python3 + Markdown, pdflatex (preview, amsmath), pdftocairo (poppler).
Run:  python3 website/build.py
SPDX-License-Identifier: MIT
"""
from __future__ import annotations

import hashlib
import html
import os
import re
import shutil
import subprocess
import tempfile

import markdown

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
OUT = os.path.join(ROOT, "docs")
EQ_DIR = os.path.join(OUT, "assets", "eq")

OWNER = "Rendo117"
REPO = "environmental-monitoring-drone"
REPO_URL = f"https://github.com/{OWNER}/{REPO}"
SITE_URL = f"https://{OWNER.lower()}.github.io/{REPO}/"
AUTHOR = "Cristian-Mihail Chirea"

NAV = [("index", "Overview"), ("math", "Mathematics"), ("hardware", "Hardware"),
       ("software", "Software"), ("simulation", "Simulation"), ("errata", "Errata")]

# ----------------------------------------------------------------------------- math
_eq_cache: dict[str, str] = {}


def _key(tex: str, display: bool) -> str:
    return hashlib.sha1(((("D" if display else "I") + tex).encode())).hexdigest()[:12]


def render_math(items: list[tuple[str, bool]]) -> None:
    """Typeset every (tex, display) pair not yet cached into assets/eq/<hash>.svg."""
    os.makedirs(EQ_DIR, exist_ok=True)
    todo = []
    seen = set()
    for tex, disp in items:
        k = _key(tex, disp)
        if k in _eq_cache or k in seen:
            continue
        seen.add(k)
        todo.append((k, tex, disp))
    if not todo:
        return
    body = []
    for _, tex, disp in todo:
        t = tex.strip()
        body.append(r"\eqsnip{$" + (r"\displaystyle " if disp else "") + t + r"$}")
    doc = (r"\documentclass[10pt]{article}\usepackage[utf8]{inputenc}\usepackage{amsmath,amssymb,textcomp}"
           r"\usepackage[active,tightpage]{preview}\setlength\PreviewBorder{0.6pt}"
           r"\newsavebox\eqbox\newcommand\eqsnip[1]{\sbox\eqbox{#1}"
           r"\typeout{EQDIM:\number\ht\eqbox:\number\dp\eqbox:\number\wd\eqbox}"
           r"\begin{preview}\usebox\eqbox\end{preview}}"
           r"\begin{document}" + "\n".join(body) + r"\end{document}")
    with tempfile.TemporaryDirectory() as td:
        with open(os.path.join(td, "eq.tex"), "w") as fh:
            fh.write(doc)
        r = subprocess.run(["pdflatex", "-interaction=nonstopmode", "eq.tex"], cwd=td,
                           capture_output=True, text=True)
        logtxt = open(os.path.join(td, "eq.log"), errors="replace").read()
        if r.returncode != 0:
            errs = re.findall(r"^! .*?(?:\n.*){0,3}", logtxt, re.M)
            raise RuntimeError("LaTeX failed:\n" + "\n".join(errs[:5]))
        dims = [(None, h, d, w) for h, d, w in re.findall(r"EQDIM:(-?\d+):(-?\d+):(-?\d+)", logtxt)]
        assert len(dims) == len(todo), (len(dims), len(todo))
        for (k, tex, disp), (_, h, d, w) in zip(todo, dims):
            i = todo.index((k, tex, disp)) + 1
            svg = os.path.join(EQ_DIR, f"{k}.svg")
            subprocess.run(["pdftocairo", "-svg", "-f", str(i), "-l", str(i),
                            os.path.join(td, "eq.pdf"), svg], check=True)
            sp = 65536.0
            hp, dp, wp = int(h) / sp, int(d) / sp, int(w) / sp
            b = 0.6
            W, H, D = (wp + 2 * b) / 10, (hp + dp + 2 * b) / 10, (dp + b) / 10
            alt = html.escape(tex.strip(), quote=True)
            style = f"width:{W:.3f}em;height:{H:.3f}em"
            if not disp:
                style += f";vertical-align:-{D:.3f}em"
            _eq_cache[k] = f'<img class="eq" src="{{ROOT}}assets/eq/{k}.svg" alt="{alt}" style="{style}">'


MATH_D = re.compile(r"\$\$(.+?)\$\$", re.S)
MATH_I = re.compile(r"(?<![\\$\w])\$(?!\$)([^$\n]+?)(?<!\\)\$(?!\w)")
CODE = re.compile(r"(```.*?```|`[^`\n]+`)", re.S)


def extract_math(text: str):
    """Replace math by placeholders (code spans are protected). Returns text, list of (tex, disp)."""
    found = []
    codes = []

    def keep_code(m):
        codes.append(m.group(0))
        return f"\x00C{len(codes) - 1}\x00"

    text = CODE.sub(keep_code, text)

    def d(m):
        found.append((m.group(1), True))
        return f"\x00D{len(found) - 1}\x00"

    def i(m):
        found.append((m.group(1), False))
        return f"\x00I{len(found) - 1}\x00"

    text = MATH_D.sub(d, text)
    text = MATH_I.sub(i, text)
    text = re.sub(r"\x00C(\d+)\x00", lambda m: codes[int(m.group(1))], text)
    return text, found


def restore_math(text: str, found) -> str:
    def rep(m):
        kind, n = m.group(1), int(m.group(2))
        tex, disp = found[n]
        img = _eq_cache[_key(tex, disp)]
        return f'<div class="display-eq">{img}</div>' if disp else img
    # a display placeholder alone in a <p> -> unwrap
    text = re.sub(r"<p>\s*(\x00D\d+\x00)\s*</p>", r"\1", text)
    return re.sub(r"\x00([DI])(\d+)\x00", rep, text)


# ----------------------------------------------------------------------------- markdown
def md_to_html(path: str) -> tuple[str, str]:
    src = open(path, encoding="utf-8").read()
    title = re.search(r"^# (.+)$", src, re.M).group(1)
    src = re.sub(r"^# .+\n", "", src, count=1, flags=re.M)
    src, found = extract_math(src)
    render_math(found)
    body = markdown.markdown(src, extensions=["tables", "fenced_code", "sane_lists"])
    body = restore_math(body, found)
    # repo-relative links -> GitHub, image paths -> site assets
    body = re.sub(r'src="\.\./docs/assets/', 'src="{ROOT}assets/', body)
    body = re.sub(r'href="\.\./docs/assets/', 'href="{ROOT}assets/', body)
    body = re.sub(r'href="\.\./([^"#]+)"', lambda m: f'href="{REPO_URL}/blob/main/{m.group(1)}"', body)
    body = re.sub(r'href="(?!https?:|#|\{ROOT\}|mailto:)([^"]+\.(?:md|py|svg|png|json|ino|h))"',
                  lambda m: f'href="{REPO_URL}/blob/main/documentation/{m.group(1)}"', body)
    body = body.replace("<table>", '<div class="table-wrap"><table>').replace("</table>", "</table></div>")
    body = (body.replace("🔴", '<span class="sev sev-h">● high</span>')
                .replace("🟠", '<span class="sev sev-m">● medium</span>')
                .replace("🟡", '<span class="sev sev-l">● low</span>')
                .replace("⚠", "<span aria-hidden=\"true\">▲</span>"))
    return title, body


# ----------------------------------------------------------------------------- template
def page(slug: str, title: str, body: str, description: str, prose: bool = True) -> str:
    root = ""  # all pages live at the site root
    nav = "\n".join(
        f'<a href="{s}.html"{" aria-current=\"page\"" if s == slug else ""}>{n}</a>' if s != "index"
        else f'<a href="./"{" aria-current=\"page\"" if s == slug else ""}>{n}</a>'
        for s, n in NAV)
    full_title = f"{title} — Environmental Monitoring Drone" if slug != "index" else "Environmental Monitoring Drone"
    content = f'<div class="prose">{body}</div>' if prose else body
    out = f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
{'<base href="/' + REPO + '/">' if slug == '404' else ''}
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{html.escape(full_title)}</title>
<meta name="description" content="{html.escape(description)}">
<meta name="author" content="{AUTHOR}">
<link rel="canonical" href="{SITE_URL}{'' if slug == 'index' else slug + '.html'}">
<meta property="og:title" content="{html.escape(full_title)}">
<meta property="og:description" content="{html.escape(description)}">
<meta property="og:type" content="website">
<meta property="og:image" content="{SITE_URL}assets/img/drone-photo.jpg">
<link rel="icon" href="{{ROOT}}assets/img/favicon.svg" type="image/svg+xml">
<link rel="stylesheet" href="{{ROOT}}assets/style.css">
<script>try{{var t=localStorage.getItem('theme');if(t)document.documentElement.setAttribute('data-theme',t);}}catch(e){{}}</script>
</head>
<body>
<a class="skip" href="#main">Skip to content</a>
<header class="bar"><div class="bar-inner">
<a class="home" href="./">Environmental Monitoring Drone</a>
<nav aria-label="Sections">
{nav}
<a class="gh" href="{REPO_URL}">GitHub</a>
</nav>
</div></header>
<main id="main">
{content}
</main>
<footer><div class="foot">
<div>© 2022–2026 {AUTHOR}. Code: MIT licence · Text and figures: CC BY 4.0.</div>
<nav aria-label="Legal">
<a href="legal.html">Legal notice</a>
<a href="privacy.html">Privacy</a>
<a href="terms.html">Terms of use</a>
<a href="legal.html#licences">Licences</a>
<button class="theme-toggle" type="button" id="theme">Toggle dark mode</button>
</nav>
</div></footer>
<script>
(function(){{var b=document.getElementById('theme');if(!b)return;b.addEventListener('click',function(){{
var r=document.documentElement,cur=r.getAttribute('data-theme');
var dark=cur?cur==='dark':window.matchMedia('(prefers-color-scheme: dark)').matches;
var n=dark?'light':'dark';r.setAttribute('data-theme',n);try{{localStorage.setItem('theme',n);}}catch(e){{}}}});}})();
</script>
</body>
</html>
"""
    # figures open full size in a new tab
    out = re.sub(r'<div class="frame"><img src="([^"]+)"', r'<div class="frame"><a href="\1"><img src="\1"', out)
    out = re.sub(r'(<div class="frame"><a href="[^"]+"><img [^>]+>)', r'\1</a>', out)
    return out.replace("{ROOT}", root)


def read_fragment(name: str):
    raw = open(os.path.join(HERE, "pages", name), encoding="utf-8").read()
    m = re.match(r"<!--\s*title:\s*(.*?)\s*\|\s*description:\s*(.*?)\s*(\|\s*wide)?\s*-->\n", raw)
    title, desc, wide = m.group(1), m.group(2), bool(m.group(3))
    body = raw[m.end():]
    body = body.replace("{REPO_URL}", REPO_URL).replace("{SITE_URL}", SITE_URL)
    body, found = extract_math(body)
    render_math(found)
    return title, desc, restore_math(body, found), not wide


def main():
    os.makedirs(OUT, exist_ok=True)
    shutil.copy(os.path.join(HERE, "style.css"), os.path.join(OUT, "assets", "style.css"))
    built = []
    for f in sorted(os.listdir(os.path.join(HERE, "pages"))):
        if not f.endswith(".html"):
            continue
        slug = f[:-5]
        title, desc, body, prose = read_fragment(f)
        with open(os.path.join(OUT, f), "w", encoding="utf-8") as fh:
            fh.write(page(slug, title, body, desc, prose))
        built.append(f)
    md_pages = {"math": ("math-model.md", "Mathematical model of the quadcopter: kinematics, Newton–Euler dynamics, X-frame allocation, linearisation, PID and estimator analysis."),
                "errata": ("errata.md", "Corrections to the 2022 thesis: firmware, Raspberry Pi script, Java application, hardware and text."),
                "calibration": ("calibration.md", "Bench set-up, sign checks, ESC and sensor calibration, first-flight checklist."),
                "safety": ("safety-and-regulations.md", "Safety notes and EU/Romanian drone regulations relevant to the project.")}
    for slug, (md, desc) in md_pages.items():
        title, body = md_to_html(os.path.join(ROOT, "documentation", md))
        body = f"<h1>{html.escape(title)}</h1>\n" + body
        with open(os.path.join(OUT, f"{slug}.html"), "w", encoding="utf-8") as fh:
            fh.write(page(slug, title, body, desc))
        built.append(f"{slug}.html")
    # unused equation files from earlier builds
    used = set(_eq_cache)
    for f in os.listdir(EQ_DIR):
        if f[:-4] not in used:
            os.remove(os.path.join(EQ_DIR, f))
    open(os.path.join(OUT, ".nojekyll"), "w").close()
    print("built:", ", ".join(built), f"| {len(_eq_cache)} equations")


if __name__ == "__main__":
    main()
