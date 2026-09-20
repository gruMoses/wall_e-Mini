"""Safety contracts for the waypoint-navigation page HTML/JS.

Covers the 2026-09-19 field-report fixes (issues #45, #46, #47, #48, #51):
fixed STOP outside the sheet, GPS gate matching the backend (RTK quality 4),
no native confirm/alert on the launch path, collapse-on-start, and the
release-does-not-stop toast.
"""

from __future__ import annotations

import unittest

from pi_app.web.waypoint_nav_ui import _NAV_HTML


class TestWaypointNavUiSafety(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.html = _NAV_HTML

    def test_fixed_stop_exists_outside_sheet_body(self):
        html = self.html
        stop_idx = html.find('id="btnStop"')
        self.assertNotEqual(stop_idx, -1, "btnStop id must still exist")
        sheet_idx = html.find('id="sheet"')
        self.assertNotEqual(sheet_idx, -1)
        body_idx = html.find('class="sheet-body"')
        self.assertNotEqual(body_idx, -1)

        # STOP is after the sheet element (placed after its closing tag).
        self.assertGreater(stop_idx, sheet_idx)

        # The sheet-body region (up to the speed slider near its end) must
        # not contain btnStop; the fixed control lives after the sheet.
        body_region = html[body_idx:html.find('id="speedSlider"')]
        self.assertNotIn('id="btnStop"', body_region)
        self.assertIn('class="nav-estop"', html)
        self.assertRegex(
            html,
            r'<button[^>]*(id="btnStop"[^>]*class="nav-estop"|class="nav-estop"[^>]*id="btnStop")',
        )

    def test_gps_gate_requires_exact_rtk_fixed(self):
        html = self.html
        self.assertIn("gpsFix === 4", html)
        self.assertNotIn("gpsFix < 4", html)

    def test_launch_path_has_no_confirm_or_alert(self):
        html = self.html
        start = html.find("function goDown")
        end = html.find("function exportRoute")
        self.assertGreater(start, 0)
        self.assertGreater(end, start)
        launch = html[start:end]
        self.assertIn("function goDown", launch)
        self.assertIn("function startNavWithWaypoints", launch)
        self.assertIn("function startNav", launch)
        self.assertNotIn("confirm(", launch)
        self.assertNotIn("alert(", launch)

    def test_release_does_not_stop_toast(self):
        self.assertIn("Releasing does NOT stop it", self.html)
        self.assertIn("Robot is moving. Releasing does NOT stop it. Use STOP.", self.html)

    def test_collapse_on_nav_start_path_exists(self):
        self.assertIn("function collapseSheetOnNavStart", self.html)
        self.assertIn("collapseSheetOnNavStart()", self.html)

    def test_sheet_default_is_collapsed_in_markup(self):
        self.assertRegex(self.html, r'class="bottom-sheet collapsed"')
        self.assertIn("max-height: 40vh", self.html)
        self.assertIn("initSheetDefaultState", self.html)

    def test_go_notice_and_nav_status_ids_exist(self):
        self.assertIn('id="goLaunchNotice"', self.html)
        self.assertIn('id="navRunStatus"', self.html)
        self.assertIn('id="btnGo"', self.html)
        self.assertIn('id="btnPause"', self.html)
        self.assertIn('id="btnSkip"', self.html)
        self.assertIn('id="sheet"', self.html)


if __name__ == "__main__":
    unittest.main()
