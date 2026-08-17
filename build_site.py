#!/usr/bin/env python3
"""把 docs/ 底下的 markdown 轉成 GitHub Pages 用的靜態 HTML。

產出的 index.html 與原始 README.md 並排放在同一個目錄:GitHub 上瀏覽 repo 看 markdown,
Pages 上瀏覽網站看 HTML,兩邊同一份來源。

執行(docker + uv 暫時環境,不裝任何東西到系統):

    docker run --rm --log-opt max-size=10m --log-opt max-file=3 \
      -v "$PWD":/w -w /w --user "$(id -u):$(id -g)" \
      -e HOME=/tmp -e UV_CACHE_DIR=/tmp/uv \
      ghcr.io/astral-sh/uv:python3.12-bookworm-slim \
      uv run --with markdown --with pygments python build_site.py

錨點的 slug 演算法刻意對齊 GitHub,讓 markdown 裡既有的 `#中文標題` 連結在網站上照樣有效。
"""
from __future__ import annotations

import re
import shutil
from pathlib import Path

import markdown

ROOT = Path(__file__).resolve().parent
DOCS = ROOT / "docs"
REPO = "https://github.com/wicanr2/isaac-sim-study"
BLOB = f"{REPO}/blob/main/"

ZONES = [
    ("common", "共通"),
    ("5.1", "5.1"),
    ("6.0.1", "6.0.1"),
]

SITE_TITLE = "Isaac Sim 實戰筆記"


# --------------------------------------------------------------------------- #
# GitHub 相容的標題 slug
# --------------------------------------------------------------------------- #
def gh_slug(text: str, _sep: str = "-") -> str:
    s = text.strip().lower()
    s = re.sub(r"[^\w\s-]", "", s, flags=re.UNICODE)
    s = re.sub(r"\s+", "-", s)
    return s


# --------------------------------------------------------------------------- #
# 檔案盤點
# --------------------------------------------------------------------------- #
class Page:
    def __init__(self, src: Path):
        self.src = src
        self.rel = src.relative_to(DOCS)
        # README.md / index.md 當成該目錄的首頁
        if src.stem in ("README", "index"):
            self.out = src.with_name("index.html")
            self.url = "" if self.rel.parent == Path(".") else f"{self.rel.parent.as_posix()}/"
        else:
            self.out = src.with_suffix(".html")
            self.url = self.out.relative_to(DOCS).as_posix()
        self.depth = len(self.out.relative_to(DOCS).parts) - 1
        self.text = src.read_text(encoding="utf-8")
        m = re.search(r"^#\s+(.+)$", self.text, re.M)
        self.title = m.group(1).strip() if m else src.stem
        # 編號取自目錄名(每篇都有),不取自 h1——只有部分 h1 寫了編號
        d = re.match(r"^(\d\d)-", src.parent.name)
        self.num = d.group(1) if d else ""
        # 側欄用短標題:去掉標題裡重複的編號,長標題砍到冒號為止
        s = re.sub(r"^\d\d\s*[·—.\-]?\s*", "", self.title)
        head = re.split(r"[::]", s)[0]
        self.short = head if len(s) > 20 and len(head) >= 5 else s

    def up(self, target: str) -> str:
        """從本頁指到站台根底下的 target。"""
        return "../" * self.depth + target


def collect() -> list[Page]:
    pages = []
    for md in sorted(DOCS.rglob("*.md")):
        pages.append(Page(md))
    return pages


# --------------------------------------------------------------------------- #
# 連結改寫:.md → 站台網址;指到 docs/ 外面的改成 GitHub 連結或降級成純文字
# --------------------------------------------------------------------------- #
def rewrite_links(html: str, page: Page) -> str:
    def fix(m: re.Match) -> str:
        href = m.group(1)
        if href.startswith(("http://", "https://", "#", "mailto:", "data:")):
            return m.group(0)
        path, _, frag = href.partition("#")
        if not path:
            return m.group(0)
        target = (page.src.parent / path).resolve()
        frag = f"#{frag}" if frag else ""

        # 指到 docs/ 裡面 → 站台內部連結
        try:
            inside = target.relative_to(DOCS)
        except ValueError:
            inside = None
        if inside is not None:
            if target.suffix == ".md":
                if target.stem in ("README", "index"):
                    dest = inside.parent.as_posix()
                    dest = "" if dest == "." else dest + "/"
                else:
                    dest = inside.with_suffix(".html").as_posix()
            else:
                dest = inside.as_posix()
            return f'href="{page.up(dest)}{frag}"'

        # 指到 repo 裡但在 docs/ 外面 → GitHub 上的檔案
        try:
            in_repo = target.relative_to(ROOT)
        except ValueError:
            in_repo = None
        if in_repo is not None and target.exists():
            return f'href="{BLOB}{in_repo.as_posix()}{frag}"'

        # 指到 repo 外面(本機才有的內部資料)→ 標記,稍後降級成純文字
        return 'href="data:offsite"'

    return re.sub(r'href="([^"]+)"', fix, html)


OFFSITE_A = re.compile(r'<a href="data:offsite">(.*?)</a>', re.S)


def demote_offsite(html: str) -> str:
    return OFFSITE_A.sub(r'<span class="offsite">\1<span class="offsite-tag">內部資料</span></span>', html)


# --------------------------------------------------------------------------- #
# 導覽
# --------------------------------------------------------------------------- #
def build_nav(pages: list[Page], page: Page) -> tuple[str, str]:
    """回傳 (頂端版本列, 側欄篇章列)。"""
    zone = page.rel.parts[0] if len(page.rel.parts) > 1 else ""

    def tab(href: str, label: str, active: bool) -> str:
        cls = ' class="on"' if active else ""
        return f'<a href="{href}"{cls}>{label}</a>'

    tabs = [tab(page.up(""), "總覽", page.url == "")]
    for z, label in ZONES:
        tabs.append(tab(page.up(z + "/"), label, zone == z))
    tabs.append(tab(page.up("version-matrix.html"), "版本差異",
                    page.rel.name == "version-matrix.md"))
    top = "".join(tabs)

    if zone not in dict(ZONES):
        return top, ""

    items = []
    for p in pages:
        if len(p.rel.parts) < 2 or p.rel.parts[0] != zone:
            continue
        if p.src.stem == "index":
            continue
        if p.src.stem != "README":
            continue  # 子頁不進側欄主列
        on = ' class="on"' if p.url == page.url else ""
        num = f'<span class="n">{p.num}</span>' if p.num else ""
        items.append(f'<li><a href="{page.up(p.url)}"{on}>{num}{p.short}</a></li>')
    side = f'<div class="side-h">{dict(ZONES)[zone]}</div><ul class="side-l">{"".join(items)}</ul>'
    return top, side


def build_toc(html: str) -> str:
    heads = re.findall(r'<h([23]) id="([^"]+)">(.*?)</h[23]>', html, re.S)
    if len(heads) < 3:
        return ""
    items = []
    for lvl, hid, txt in heads:
        clean = re.sub(r"<[^>]+>", "", txt)
        items.append(f'<li class="l{lvl}"><a href="#{hid}">{clean}</a></li>')
    return f'<div class="side-h">本篇</div><ul class="toc">{"".join(items)}</ul>'


# --------------------------------------------------------------------------- #
# 版面
# --------------------------------------------------------------------------- #
def render(page: Page, body: str, top: str, side: str) -> str:
    title = page.title if page.url else SITE_TITLE
    full = f"{title} — {SITE_TITLE}" if page.url else SITE_TITLE
    home = page.up("")
    return f"""<!doctype html>
<html lang="zh-Hant">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{full}</title>
<link rel="stylesheet" href="{page.up("style.css")}">
</head>
<body>
<header class="top">
  <a class="brand" href="{home}">{SITE_TITLE}</a>
  <nav class="tabs">{top}</nav>
  <a class="gh" href="{REPO}">GitHub</a>
</header>
<div class="shell">
  <aside class="side">{side}</aside>
  <main class="doc">
{body}
  </main>
</div>
<footer class="foot">
  <a href="{home}">總覽</a> · <a href="{page.up("version-matrix.html")}">版本差異速查</a> · <a href="{REPO}">原始碼</a>
</footer>
</body>
</html>
"""


CSS = """
:root{
  --bg:#fbfbfa; --fg:#1a1a19; --muted:#6b6b66; --line:#e0e0dc; --line2:#efefec;
  --accent:#0b6b5f; --bad:#b4341f; --warn:#8a6100; --code:#f3f3f0;
  --mono:ui-monospace,SFMono-Regular,"SF Mono",Menlo,Consolas,monospace;
  --sans:-apple-system,BlinkMacSystemFont,"Segoe UI","Noto Sans TC","PingFang TC",sans-serif;
}
@media (prefers-color-scheme:dark){
  :root:not([data-theme="light"]){
    --bg:#16171a; --fg:#e8e8e4; --muted:#9a9a94; --line:#2c2e33; --line2:#212328;
    --accent:#4fd1bd; --bad:#ff8a70; --warn:#e0b050; --code:#1d1f23;
  }
}
:root[data-theme="dark"]{
  --bg:#16171a; --fg:#e8e8e4; --muted:#9a9a94; --line:#2c2e33; --line2:#212328;
  --accent:#4fd1bd; --bad:#ff8a70; --warn:#e0b050; --code:#1d1f23;
}
*{box-sizing:border-box}
html{scroll-behavior:smooth;scroll-padding-top:4.5rem}
body{margin:0;background:var(--bg);color:var(--fg);font:16px/1.75 var(--sans);
  -webkit-font-smoothing:antialiased}

/* 頂欄:一條細線,不做卡片、不做陰影 */
.top{position:sticky;top:0;z-index:20;display:flex;align-items:center;gap:1.5rem;
  padding:.7rem 1.5rem;background:var(--bg);border-bottom:1px solid var(--line);
  max-width:100vw;overflow:hidden}
.brand{font-weight:650;letter-spacing:-.01em;color:var(--fg);text-decoration:none;
  border-bottom:0;white-space:nowrap}
.tabs{display:flex;gap:1.25rem;flex:1;min-width:0;overflow-x:auto;scrollbar-width:none}
.tabs::-webkit-scrollbar{display:none}
.tabs a{color:var(--muted);text-decoration:none;font-size:.92rem;white-space:nowrap;
  padding-bottom:.15rem;border-bottom:2px solid transparent}
.tabs a:hover{color:var(--fg)}
.tabs a.on{color:var(--fg);border-bottom-color:var(--accent)}
.gh{color:var(--muted);text-decoration:none;border-bottom:0;font-size:.85rem;white-space:nowrap}
.gh:hover{color:var(--fg)}

.shell{display:flex;gap:3rem;max-width:1240px;margin:0 auto;padding:0 1.5rem;align-items:flex-start}
.side{position:sticky;top:4.2rem;width:15rem;flex:none;padding:2rem 0 3rem;
  max-height:calc(100vh - 4.2rem);overflow-y:auto;font-size:.88rem}
.side-h{color:var(--muted);font-size:.75rem;letter-spacing:.08em;text-transform:uppercase;
  margin:0 0 .7rem;padding-bottom:.4rem;border-bottom:1px solid var(--line)}
.side-h~.side-h{margin-top:2.2rem}
.side-l,.toc{list-style:none;margin:0;padding:0}
.side-l li{margin:.1rem 0}
.side-l a{display:flex;gap:.55rem;color:var(--muted);text-decoration:none;border-bottom:0;
  padding:.28rem 0;line-height:1.45}
.side-l a:hover{color:var(--fg)}
.side-l a.on{color:var(--accent);font-weight:600}
.side-l .n{color:color-mix(in srgb,var(--muted) 45%,transparent);
  font-variant-numeric:tabular-nums;flex:none}
.side-l a.on .n,.side-l a:hover .n{color:var(--muted)}
.toc li{margin:.05rem 0}
.toc a{display:block;color:var(--muted);text-decoration:none;border-bottom:0;
  padding:.2rem 0;line-height:1.4}
.toc a:hover{color:var(--fg)}
.toc .l3{padding-left:.9rem;font-size:.94em}

.doc{flex:1;min-width:0;max-width:47rem;padding:2rem 0 6rem}

/* 標題:靠字級與留白建立階層 */
h1{font-size:1.85rem;line-height:1.3;letter-spacing:-.015em;margin:.5rem 0 1.6rem;font-weight:680}
h2{font-size:1.24rem;line-height:1.4;margin:3.2rem 0 1rem;padding-top:1.5rem;
  border-top:1px solid var(--line);font-weight:650;letter-spacing:-.01em}
h3{font-size:1.02rem;margin:2.2rem 0 .7rem;font-weight:650}
h4{font-size:.95rem;margin:1.6rem 0 .5rem;color:var(--muted);font-weight:650}
h1+p,h2+p,h3+p{margin-top:0}
p{margin:1.05rem 0}
a{color:var(--accent);text-decoration:none;border-bottom:1px solid color-mix(in srgb,var(--accent) 30%,transparent)}
a:hover{border-bottom-color:var(--accent)}
strong{font-weight:650}
hr{border:0;border-top:1px solid var(--line);margin:2.6rem 0}

ul,ol{padding-left:1.35rem;margin:1.05rem 0}
li{margin:.35rem 0}
li>ul,li>ol{margin:.35rem 0}

blockquote{margin:1.4rem 0;padding:.1rem 0 .1rem 1.15rem;border-left:2px solid var(--line);color:var(--muted)}
blockquote strong{color:var(--fg)}

code{font-family:var(--mono);font-size:.875em;background:var(--code);padding:.12em .38em;
  border-radius:3px;word-break:break-word}
pre{background:var(--code);border:1px solid var(--line2);border-radius:6px;padding:.9rem 1.1rem;
  overflow-x:auto;margin:1.4rem 0;line-height:1.6}
pre code{background:none;padding:0;font-size:.855rem}

.tw{overflow-x:auto;margin:1.5rem 0;border-top:1px solid var(--line);border-bottom:1px solid var(--line)}
table{border-collapse:collapse;width:100%;font-size:.9rem;line-height:1.6}
th,td{text-align:left;vertical-align:top;padding:.6rem .9rem;border-bottom:1px solid var(--line2)}
th{font-weight:650;color:var(--muted);font-size:.8rem;letter-spacing:.03em;white-space:nowrap;
  border-bottom:1px solid var(--line)}
tr:last-child td{border-bottom:0}
td:first-child,th:first-child{padding-left:0}
td:last-child,th:last-child{padding-right:0}

img,svg{max-width:100%;height:auto}
p[align="center"]{margin:2rem 0;text-align:center}
/* 既有圖解都是硬編白底,深色模式下給它一個明確的圖框,免得看起來像破圖 */
.doc p[align="center"] img{background:#fff;border:1px solid var(--line);border-radius:6px;padding:.5rem}

.offsite{color:var(--muted)}
.offsite-tag{font-size:.72rem;color:var(--muted);border:1px solid var(--line);border-radius:3px;
  padding:.05rem .3rem;margin-left:.3rem;white-space:nowrap}

.foot{max-width:1240px;margin:0 auto;padding:1.5rem;border-top:1px solid var(--line);
  color:var(--muted);font-size:.85rem}
.foot a{color:var(--muted);border-bottom:0}
.foot a:hover{color:var(--fg)}

@media (max-width:900px){
  .shell{display:block;padding:0 1.15rem}
  .side{position:static;width:auto;max-height:none;padding:1.5rem 0 0;overflow:visible}
  .doc{max-width:none;padding-top:1.5rem}
  .toc{display:none}
  h1{font-size:1.55rem}
  .top{padding:.6rem 1.15rem;gap:.9rem}
  .gh{display:none}
  .brand{font-size:.92rem}
}
"""


# --------------------------------------------------------------------------- #
def main() -> None:
    pages = collect()
    md = markdown.Markdown(
        extensions=["tables", "fenced_code", "toc", "attr_list", "sane_lists", "md_in_html"],
        extension_configs={"toc": {"slugify": gh_slug, "permalink": False}},
    )

    for page in pages:
        md.reset()
        body = md.convert(page.text)
        body = rewrite_links(body, page)
        body = demote_offsite(body)
        body = re.sub(r"<table>", '<div class="tw"><table>', body)
        body = re.sub(r"</table>", "</table></div>", body)
        top, side = build_nav(pages, page)
        side = side + build_toc(body)
        page.out.write_text(render(page, body, top, side), encoding="utf-8")
        print(f"  {page.out.relative_to(ROOT)}")

    (DOCS / "style.css").write_text(CSS.lstrip(), encoding="utf-8")
    (DOCS / ".nojekyll").write_text("", encoding="utf-8")
    print(f"\n{len(pages)} 頁 + style.css + .nojekyll")


if __name__ == "__main__":
    main()
