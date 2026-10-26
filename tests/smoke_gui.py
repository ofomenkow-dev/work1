"""Run from project root: QT_QPA_PLATFORM=offscreen PYTHONPATH=. python tests/smoke_gui.py"""
import os
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch
from PySide6.QtWidgets import QApplication,QMessageBox,QFileDialog
from robot_lab.app import Window,STYLE
from robot_lab.model import Config

app=QApplication([])
app.setStyle('Fusion')
app.setStyleSheet(STYLE)
w=Window()
w.show()
app.processEvents()
with patch.object(QMessageBox,'warning') as warning, TemporaryDirectory() as tmp:
    for kind in ('scara','cartesian','cylindrical'):
        for mode,path in (('position','line'),('contour','line'),('contour','circle')):
            w.populate(Config(kind=kind,mode=mode,path=path,center_y=100))
            w.build_workspace()
            w.run_simulation()
            assert w.result is not None,(kind,mode,path)
            assert w.export_button.isEnabled()
            w.toggle_animation()
            app.processEvents()
            w.animate()
            w.stop_animation()
    warning.assert_not_called()
    w.populate(Config())
    w.build_workspace()
    w.run_simulation()
    app.processEvents()
    target=Path(os.environ.get('ROBOT_SCREENSHOT',str(Path(tmp)/'robot-lab.png')))
    assert w.grab().save(str(target))
    project=Path(tmp)/'project.json'
    with patch.object(QFileDialog,'getSaveFileName',return_value=(str(project),'')):
        w.save_project()
    assert Config.load(project)==Config()
    csv=Path(tmp)/'results.csv'
    with patch.object(QFileDialog,'getSaveFileName',return_value=(str(csv),'')):
        w.export_csv()
    assert len(csv.read_text(encoding='utf-8-sig').splitlines())==402
    with patch.object(QFileDialog,'getOpenFileName',return_value=(str(project),'')):
        w.load_project()
    assert w.result is None
    w.run_simulation()
    w.controls['l1'].setValue(360)
    assert w.result is None
    assert not w.export_button.isEnabled()
    assert w.metrics[0].text()=='—'
    w.populate(Config(end_x=9000))
    w.run_simulation()
    warning.assert_called_once()
w.close()
app.processEvents()
print('GUI smoke passed: 3 schemes × 3 workflows; animation, JSON, CSV, invalidation, rejection, rendering.')
