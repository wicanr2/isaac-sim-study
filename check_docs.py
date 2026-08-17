#!/usr/bin/env python3
"""教學文件的一致性檢查。

跑法(不需要任何第三方套件):

    python3 check_docs.py

檢查五件事,全部是這個 repo 實際踩過的漂移:

1. 所有本地連結解析得到(搬目錄時最容易靜默壞掉)
2. 索引宣稱的篇數 == 實際篇數(每加一章就會漂一次)
3. 每一篇都被所屬區的 index.md 與根 README 收錄(新增章節時漏登)
4. `§N` 形式的章節引用指到存在的章節(重編號之後留下的懸空引用)
5. 產出的 HTML 不比 markdown 舊(改完忘記重跑 build_site.py)

**第 4 項只能證明該章節存在,不能證明引對了。** 引到「存在但講的是別的東西」
的章節,只有讀才抓得到——這個 repo 發生過兩次。
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
DOCS = ROOT / "docs"
ZONES = ["common", "5.1", "6.0.1", "fleet"]

problems: list[str] = []


def fail(msg: str) -> None:
    problems.append(msg)


def docs_in(zone: str) -> list[Path]:
    return sorted(d for d in (DOCS / zone).iterdir()
                  if d.is_dir() and re.match(r"^\d\d-", d.name))


# --------------------------------------------------------------------------- #
# 1. 連結解析
# --------------------------------------------------------------------------- #
md_link = re.compile(r"\[[^\]]*\]\(([^)\s]+)\)")
img_src = re.compile(r'<img[^>]*\ssrc="([^"]+)"')

n_links = 0
for md in sorted(ROOT.rglob("*.md")):
    if ".git" in md.parts:
        continue
    text = md.read_text(encoding="utf-8")
    for target in md_link.findall(text) + img_src.findall(text):
        if target.startswith(("http://", "https://", "#", "mailto:")):
            continue
        path = target.split("#")[0]
        if not path:
            continue
        n_links += 1
        if not (md.parent / path).resolve().exists():
            fail(f"斷鏈  {md.relative_to(ROOT)}  ->  {target}")

# --------------------------------------------------------------------------- #
# 2. 篇數
# --------------------------------------------------------------------------- #
counts = {z: len(docs_in(z)) for z in ZONES}
total = sum(counts.values())

home = (DOCS / "index.md").read_text(encoding="utf-8")
for zone, label in [("common", "共通:機制與方法論"), ("5.1", "Isaac Sim 5.1"),
                    ("6.0.1", "Isaac Sim 6.0.1"), ("fleet", "車隊與多樓層")]:
    m = re.search(rf"\[{re.escape(label)}\]\([^)]*\)\*\* · (\d+) 篇", home)
    if not m:
        fail(f"篇數  docs/index.md 找不到 {label} 的篇數宣告")
    elif int(m.group(1)) != counts[zone]:
        fail(f"篇數  docs/index.md 說 {label} 有 {m.group(1)} 篇,實際 {counts[zone]} 篇")

for zone in ZONES:
    zone_index = (DOCS / zone / "index.md").read_text(encoding="utf-8")
    m = re.search(r"這一區的 (\d+) 篇", zone_index)
    if m and int(m.group(1)) != counts[zone]:
        fail(f"篇數  docs/{zone}/index.md 說 {m.group(1)} 篇,實際 {counts[zone]} 篇")

readme = (ROOT / "README.md").read_text(encoding="utf-8")
m = re.search(r"^(\d+) 篇裡真正綁死版本", readme, re.M)
if m and int(m.group(1)) != total:
    fail(f"篇數  README.md 說總共 {m.group(1)} 篇,實際 {total} 篇")
for zone in ZONES:
    m = re.search(rf"\|\s*\[`docs/{re.escape(zone)}/`\]\([^)]*\)\s*\|\s*(\d+)\s*\|", readme)
    if m and int(m.group(1)) != counts[zone]:
        fail(f"篇數  README.md 說 {zone} 區有 {m.group(1)} 篇,實際 {counts[zone]} 篇")

# --------------------------------------------------------------------------- #
# 3. 收錄
# --------------------------------------------------------------------------- #
for zone in ZONES:
    idx = (DOCS / zone / "index.md").read_text(encoding="utf-8")
    for d in docs_in(zone):
        if d.name not in idx:
            fail(f"漏登  {d.name} 不在 docs/{zone}/index.md 裡")
        if f"docs/{zone}/{d.name}" not in readme:
            fail(f"漏登  {d.name} 不在根 README.md 裡")

# --------------------------------------------------------------------------- #
# 4. §N 章節引用
# --------------------------------------------------------------------------- #
def sections_of(md: Path) -> set[str]:
    return {m.group(1) for m in
            re.finditer(r"^#{2,3} (\d+(?:\.\d+)?)[.、·\s]", md.read_text(encoding="utf-8"), re.M)}


all_md = [md for md in DOCS.rglob("*.md")]
sec_cache = {md: sections_of(md) for md in all_md}

for md in sorted(all_md):
    text = md.read_text(encoding="utf-8")
    own = sec_cache[md]
    def check_cross(num: str, rel: str) -> None:
        tgt = (md.parent / rel).resolve()
        if tgt.suffix == ".md" and tgt in sec_cache and num not in sec_cache[tgt]:
            fail(f"章節  {md.relative_to(ROOT)} 引用 {tgt.name} 的 §{num},該篇沒有這一節")

    # 跨篇兩種寫法:章節號在連結內、章節號跟在連結後面
    for m in re.finditer(r"\[[^\]]*§(\d+(?:\.\d+)?)[^\]]*\]\(([^)\s#]+)", text):
        check_cross(m.group(1), m.group(2))
    after_link = re.compile(r"\[[^\]]*\]\(([^)\s#]+)\)\s*§(\d+(?:\.\d+)?)")
    for m in after_link.finditer(text):
        check_cross(m.group(2), m.group(1))

    # 篇內:剩下的 §N。先拿掉所有指涉「別的文件」的寫法,否則會誤判成本篇的章節
    stripped = after_link.sub(" ", text)
    stripped = re.sub(r"\[[^\]]*\]\([^)]*\)", " ", stripped)      # 連結
    stripped = re.sub(r"`[^`]*\.md`[^。\n]*", " ", stripped)          # `別的檔案.md` … §N
    stripped = re.sub(r"doc \d+[^。\n]*", " ", stripped)              # doc 70 §7.1
    for m in re.finditer(r"§(\d+)(?![\d.])", stripped):
        if own and m.group(1) not in own:
            fail(f"章節  {md.relative_to(ROOT)} 提到 §{m.group(1)},但本篇沒有這一節")

# --------------------------------------------------------------------------- #
# 5. 產出是否過期
# --------------------------------------------------------------------------- #
stale = []
for md in sorted(all_md):
    html = md.with_name("index.html") if md.stem in ("README", "index") else md.with_suffix(".html")
    if not html.exists():
        fail(f"未建  {md.relative_to(ROOT)} 沒有對應的 HTML,跑 build_site.py")
    elif html.stat().st_mtime < md.stat().st_mtime:
        stale.append(md.relative_to(ROOT))
if stale:
    fail(f"過期  {len(stale)} 個頁面的 HTML 比 markdown 舊,跑 build_site.py:"
         + "".join(f"\n        {s}" for s in stale[:5])
         + ("\n        …" if len(stale) > 5 else ""))

# --------------------------------------------------------------------------- #
print("連結 %d 條 · 篇數 %s = %d"
      % (n_links, " / ".join(f"{z} {counts[z]}" for z in ZONES), total))
if problems:
    print(f"\n{len(problems)} 個問題:")
    for p in problems:
        print(f"  {p}")
    sys.exit(1)
print("全部通過。")
