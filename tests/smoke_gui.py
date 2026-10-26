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
        for mode,path in (('position','line'),('contour','line'),('contour','circle'),('position','table'),('contour','table')):
            w.populate(Config(kind=kind,mode=mode,path=path,center_y=100))
            w.build_workspace()
            w.run_simulation()
            assert w.result is not None,(kind,mode,path)
            assert w.export_button.isEnabled()
            w.toggle_animation()
            app.processEvents()
            w.animate()
            w.stop_animation()
    for kind in ('scara','cartesian','cylindrical'):
        w.populate(Config(kind=kind,coordinate_mode='joint',path='table'))
        w.example_points()
        w.run_simulation()
        assert w.result is not None,kind
        assert w.path_plot.ax.name=='rectilinear'
        assert w.workspace_plot.ax.name=='rectilinear'
        assert w.point_table.horizontalHeaderItem(0).text().startswith('q₁')
        w.toggle_animation()
        assert len(w.trace_artist.get_xdata())==1
        w.stop_animation()
        w.animation_index=40
        w.animate()
        assert len(w.trace_artist.get_xdata())==41
        assert len(w.robot_artist.get_xdata())>=3
        assert w.progress.value()>0
        w.animation_index=len(w.result.samples)-1
        w.animate()
        assert w.progress.value()==100
        assert not w.timer.isActive()
    warning.assert_not_called()
    w.populate(Config())
    w.build_workspace()
    w.run_simulation()
    app.processEvents()
    target=Path(os.environ.get('ROBOT_SCREENSHOT',str(Path(tmp)/'robot-lab.png')))
    assert w.grab().save(str(target))
    # Table editing accepts decimal comma, add/remove, and invalidates export.
    n=w.point_table.rowCount()
    w.add_point()
    assert w.point_table.rowCount()==n+1
    w.point_table.selectRow(n)
    w.remove_point()
    assert w.point_table.rowCount()==n
    w.point_table.item(0,0).setText('280,0')
    assert w.read_config().points[0][0]==280
    assert w.result is None
    w.run_simulation()
    w.point_table.scrollToTop()
    w.form_scroll.ensureWidgetVisible(w.table_group)
    app.processEvents()
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
    w.populate(Config(path='line',mode='position',end_x=9000))
    w.run_simulation()
    warning.assert_called_once()
w.close()
app.processEvents()
print('GUI smoke passed: horizontal XY, 3 schemes, 15 motion workflows, joint tables, growing trace, heading, table edits, JSON, CSV, rejection.')
