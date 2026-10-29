"""웹을 하나로: 자비스 화면에 '주행' 탭(주행 화면 iframe)을 넣었다(2026-10-29). 구조·동작 규칙만 확인한다(브라우저 시험은 수동)."""
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
HTML = (ROOT / 'web/index.html').read_text()


def test_header_has_two_tabs_and_a_drive_panel():
    assert 'id="tab-arm"' in HTML and 'id="tab-drive"' in HTML and 'role="tablist"' in HTML
    assert 'id="view-arm"' in HTML and 'id="view-drive"' in HTML and 'id="drive-frame"' in HTML
    assert re.search(r'<iframe id="drive-frame"[^>]*src="about:blank"', HTML)                    # 처음에는 불러오지 않는다
    assert 'drive-off' in HTML and '새 창으로 열기' in HTML                                       # 주행 서버가 꺼졌을 때 안내·직접 열기


def test_drive_page_is_mounted_only_while_the_tab_is_open():
    js = HTML[HTML.index('// ---------- 탭: 자비스 / 주행'):]
    assert 'const DRIVE_PORT = 8082' in js
    assert "frame.src = 'about:blank'" in js                                                    # 나가면 내린다(0.2초 갱신이 안 돈다)
    assert "mode: 'no-cors'" in js                                                              # 주행 서버가 켜져 있는지만 본다
    assert "location.hash === '#drive'" in js and "localStorage.getItem('armTab')" in js        # 직접 링크·마지막 탭 기억
    assert "'tab-drive').classList.toggle('live'" in HTML                                       # 이동 중이면 주행 탭에 점


def test_stop_button_also_stops_manual_driving_on_the_drive_tab():
    stop = HTML[HTML.index("$('stop').onclick"):].split('\n')[0:3]
    assert any("api/drive/stop" in line for line in stop)


def test_start_all_opens_one_window_only():
    script = (ROOT / 'pi/start_all.sh').read_text()
    assert 'xdg-open' not in script and 'open_jarvis.sh" | sed' in script
    assert '위쪽 \'주행\' 탭' in script
