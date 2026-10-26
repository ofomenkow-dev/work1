"""Russian-language Qt desktop application for the independent teaching model."""
import os
import sys
import math
from pathlib import Path
os.environ.setdefault('MPLCONFIGDIR',str(Path.home()/'.robot-lab'/'matplotlib'))
from PySide6.QtCore import Qt, QLocale, QTimer
from PySide6.QtWidgets import (QApplication,QMainWindow,QWidget,QHBoxLayout,QVBoxLayout,
    QLabel,QPushButton,QComboBox,QDoubleSpinBox,QSpinBox,QFormLayout,QGroupBox,
    QTabWidget,QScrollArea,QFileDialog,QMessageBox,QSplitter,QFrame,QProgressBar,
    QTableWidget,QTableWidgetItem,QHeaderView,QAbstractItemView)
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg, NavigationToolbar2QT
from matplotlib.figure import Figure
from matplotlib.patches import FancyArrowPatch
from .model import Config, forward, inverse, workspace, simulate, tool_angle, desired, joint_pair

NAMES={'scara':'СКАРА','cartesian':'Декартовая','cylindrical':'Цилиндрическая'}
STYLE='''
QWidget {font-family: "Segoe UI", "DejaVu Sans", sans-serif; font-size: 13px; color: #243449;}
QMainWindow, QTabWidget::pane {background: #f3f6fa;}
QFrame#sidebar {background: #112539;}
QFrame#sidebar QLabel {color: #aec4d5;}
QFrame#sidebar QLabel#brand {color: white; font-size: 26px; font-weight: 700;}
QLabel#title {font-size: 26px; font-weight: 700; color: #132b43;}
QLabel#subtitle {color: #62768b;}
QPushButton {background: white; border: 1px solid #d8e2ec; border-radius: 7px; padding: 9px 14px;}
QPushButton:hover {background: #e8f3f4; border-color: #14a593;}
QPushButton:disabled {color: #9caab8; background: #edf1f5;}
QPushButton#primary {background: #087f75; color: white; border: none; font-weight: 600;}
QFrame#sidebar QPushButton {background: #1d384f; color: #eaf4fa; border: 1px solid #355168; text-align: left;}
QGroupBox {background: white; border: 1px solid #dfe7ef; border-radius: 9px; margin-top: 18px; padding: 18px 12px 12px; font-weight: 600;}
QGroupBox::title {subcontrol-origin: margin; left: 12px; padding: 0 5px;}
QDoubleSpinBox, QSpinBox, QComboBox {background: #f8fafc; border: 1px solid #d6e1eb; border-radius: 5px; padding: 6px; min-width: 80px;}
QTabBar::tab {padding: 12px 20px; background: #e8eef4; border: none; color: #5c7085;}
QTabBar::tab:selected {background: white; color: #087f75; border-bottom: 3px solid #087f75;}
QLabel#metric {font-size: 24px; font-weight: 700; color: #087f75;}
QLabel#status {padding: 9px; border-radius: 6px; background: #e4f1ee; color: #17675f;}
QScrollArea {border: none; background: transparent;}
'''


class Plot(QWidget):
    def __init__(self,projection=None):
        super().__init__()
        self.figure=Figure(figsize=(7,5),facecolor='white',layout='constrained')
        self.canvas=FigureCanvasQTAgg(self.figure)
        self.ax=self.figure.add_subplot(111,projection=projection)
        layout=QVBoxLayout(self)
        layout.setContentsMargins(0,0,0,0)
        layout.addWidget(NavigationToolbar2QT(self.canvas,self))
        layout.addWidget(self.canvas)

    def refresh(self):
        self.ax.tick_params(labelsize=9,colors='#62768b')
        self.canvas.draw_idle()


class Window(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('ROBOT Lab — Моделирование манипуляторов')
        self.resize(1400,920)
        self.setMinimumSize(1000,700)
        self.config=Config()
        self.result=None
        self.controls={}
        self.animation_index=0
        self.timer=QTimer(self)
        self.timer.setInterval(40)
        self.timer.timeout.connect(self.animate)
        root=QWidget()
        self.setCentralWidget(root)
        layout=QHBoxLayout(root)
        layout.setContentsMargins(0,0,0,0)
        layout.setSpacing(0)
        side=QFrame()
        side.setObjectName('sidebar')
        side.setFixedWidth(210)
        sl=QVBoxLayout(side)
        sl.setContentsMargins(22,30,22,24)
        badge=QLabel('ВИРТУАЛЬНАЯ\nЛАБОРАТОРИЯ')
        sl.addWidget(badge)
        brand=QLabel('ROBOT Lab')
        brand.setObjectName('brand')
        sl.addWidget(brand)
        sl.addWidget(QLabel('Кинематика\nУправление\nТочность'))
        sl.addSpacing(22)
        for text,callback in [('Открыть проект',self.load_project),('Сохранить проект',self.save_project),
                              ('Экспорт CSV',self.export_csv),('Методика расчёта',self.help)]:
            button=QPushButton(text)
            button.clicked.connect(callback)
            sl.addWidget(button)
            if text=='Экспорт CSV':
                self.export_button=button
                button.setEnabled(False)
        sl.addStretch()
        sl.addWidget(QLabel('Локальная работа\nБез подключения к сети'))
        layout.addWidget(side)
        content=QWidget()
        cl=QVBoxLayout(content)
        cl.setContentsMargins(28,24,28,18)
        cl.setSpacing(14)
        title=QLabel('Моделирование манипуляторов')
        title.setObjectName('title')
        cl.addWidget(title)
        subtitle=QLabel('Задайте пары координат или углов. Манипулятор вычертит след в горизонтальной плоскости XY.')
        subtitle.setWordWrap(True)
        subtitle.setObjectName('subtitle')
        cl.addWidget(subtitle)
        splitter=QSplitter(Qt.Horizontal)
        scroll=QScrollArea()
        self.form_scroll=scroll
        scroll.setWidgetResizable(True)
        scroll.setMinimumWidth(310)
        form=QWidget()
        self.form_layout=QVBoxLayout(form)
        self.form_layout.setContentsMargins(0,0,8,0)
        self.make_forms()
        scroll.setWidget(form)
        splitter.addWidget(scroll)
        right=QWidget()
        rl=QVBoxLayout(right)
        rl.setContentsMargins(6,0,0,0)
        self.tabs=QTabWidget()
        self.workspace_plot=Plot()
        self.path_plot=Plot()
        self.error_plot=Plot()
        self.signals_plot=Plot()
        for plot,name in [(self.workspace_plot,'Область XY'),(self.path_plot,'След XY'),
                          (self.error_plot,'Погрешность'),(self.signals_plot,'Осциллограммы')]:
            self.tabs.addTab(plot,name)
        rl.addWidget(self.tabs,1)
        metrics=QHBoxLayout()
        self.metrics=[]
        for name in ('Максимум, мм','СКО, мм','В конце, мм'):
            box=QGroupBox(name)
            bl=QVBoxLayout(box)
            value=QLabel('—')
            value.setObjectName('metric')
            bl.addWidget(value)
            metrics.addWidget(box)
            self.metrics.append(value)
        rl.addLayout(metrics)
        buttons=QHBoxLayout()
        build=QPushButton('Построить область')
        build.clicked.connect(self.build_workspace)
        run=QPushButton('Рассчитать движение')
        run.setObjectName('primary')
        run.clicked.connect(self.run_simulation)
        self.play=QPushButton('▶ Анимация')
        self.play.setEnabled(False)
        self.play.clicked.connect(self.toggle_animation)
        for b in (build,run,self.play):
            buttons.addWidget(b)
        rl.addLayout(buttons)
        self.progress=QProgressBar()
        self.progress.setRange(0,100)
        self.progress.setValue(0)
        self.progress.setFormat('Воспроизведение: %p%')
        rl.addWidget(self.progress)
        splitter.addWidget(right)
        splitter.setSizes([335,800])
        cl.addWidget(splitter,1)
        self.status=QLabel('Готово. Выберите параметры и выполните расчёт.')
        self.status.setObjectName('status')
        self.status.setWordWrap(True)
        cl.addWidget(self.status)
        layout.addWidget(content,1)
        self.populate(self.config)
        self.build_workspace()

    def group(self,title):
        box=QGroupBox(title)
        form=QFormLayout(box)
        form.setLabelAlignment(Qt.AlignLeft)
        form.setRowWrapPolicy(QFormLayout.WrapAllRows)
        form.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow)
        self.form_layout.addWidget(box)
        return form

    def number(self,form,key,title,lo,hi,decimals=2,integer=False):
        w=QSpinBox() if integer else QDoubleSpinBox()
        w.setLocale(QLocale('ru_RU'))
        w.setRange(lo,hi)
        if not integer:
            w.setDecimals(decimals)
            w.setSingleStep(.001 if key=='dt' else .01 if key=='tau' else 1)
        w.setKeyboardTracking(False)
        w.valueChanged.connect(self.invalidate)
        self.controls[key]=w
        form.addRow(title,w)

    def combo(self,form,key,title,options):
        w=QComboBox()
        for text,code in options:
            w.addItem(text,code)
        w.currentIndexChanged.connect(self.invalidate)
        self.controls[key]=w
        form.addRow(title,w)

    def make_forms(self):
        f=self.group('01   Кинематическая схема')
        self.combo(f,'kind','Тип робота',[(v,k) for k,v in NAMES.items()])
        self.controls['kind'].currentIndexChanged.connect(self.update_fields)
        self.geometry=self.group('04   Геометрия и ограничения')
        specs=[('l1','Звено L₁, мм',1,10000),('l2','Звено L₂, мм',1,10000),('l3','Инструмент L₃, мм',0,10000),
               ('q1min','q₁ минимум, °',-180,179),('q1max','q₁ максимум, °',-179,180),
               ('q2min','q₂ минимум, °',-180,179),('q2max','q₂ максимум, °',-179,180),
               ('rmin','Радиус min, мм',0,9999),('rmax','Радиус max, мм',1,10000),
               ('xmax','Ход X, мм',1,10000),('ymax','Ход Y, мм',1,10000),('zmax','Ход Z, мм',1,10000)]
        for key,title,lo,hi in specs:
            self.number(self.geometry,key,title,lo,hi)
        f=self.group('05   Управление и вычисления')
        for key,title,lo,hi,d in [('kp','Усиление Kp',.01,1000,2),('tau','Постоянная τ, с',.001,100,3),
            ('dt','Шаг расчёта, с',.001,.1,3),('frequency','Контроллер, Гц',1,10000,1),
            ('speed','Линейная скорость max, мм/с',.1,10000,1),('angular_speed','Угловая скорость max, °/с',.1,10000,1)]:
            self.number(f,key,title,lo,hi,d)
        self.number(f,'bits','Разрядность АЦП/ЦАП, бит',4,24,integer=True)
        self.number(f,'cycles','Задержка, тактов',0,100,integer=True)
        self.motion=self.group('02   Задание в плоскости XY')
        self.combo(self.motion,'mode','Режим',[('Позиционный','position'),('Контурный','contour')])
        self.combo(self.motion,'path','Задание',[('Таблица пар','table'),('Отрезок','line'),('Окружность','circle')])
        self.combo(self.motion,'coordinate_mode','Формат таблицы',
                   [('Координаты X / Y, мм','cartesian'),('Суставные координаты q₁ / q₂','joint')])
        for key,title in [('start_x','Начало X, мм'),('start_y','Начало Y, мм'),('start_z','Высота плоскости Z, мм'),
                          ('end_x','Конец X, мм'),('end_y','Конец Y, мм'),
                          ('center_x','Центр круга X, мм'),('center_y','Центр круга Y, мм')]:
            self.number(self.motion,key,title,-20000,20000)
        self.number(self.motion,'orientation','Ориентация схвата φ в XY, °',-180,180)
        self.number(self.motion,'radius','Радиус круга, мм',.1,10000)
        self.number(self.motion,'duration','Длительность, с',.002,60,3)
        self.controls['mode'].currentIndexChanged.connect(self.update_fields)
        self.controls['path'].currentIndexChanged.connect(self.update_fields)
        self.controls['coordinate_mode'].currentIndexChanged.connect(self.update_fields)
        self.table_group=QGroupBox('03   Последовательность пар')
        tl=QVBoxLayout(self.table_group)
        self.point_table=QTableWidget(0,2)
        self.point_table.setMinimumHeight(220)
        self.point_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.point_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.point_table.cellChanged.connect(self.invalidate)
        tl.addWidget(self.point_table)
        row=QHBoxLayout()
        for text,callback in [('＋ Строка',self.add_point),('− Строка',self.remove_point),('Пример',self.example_points)]:
            b=QPushButton(text)
            b.setStyleSheet('padding: 6px 8px; font-size: 12px;')
            b.clicked.connect(callback)
            row.addWidget(b)
        tl.addLayout(row)
        self.form_layout.addWidget(self.table_group)
        self.form_layout.insertWidget(1,self.motion.parentWidget())
        self.form_layout.insertWidget(2,self.table_group)
        note=QLabel('Вид сверху: X вправо, Y вверх; Z фиксирована. Стрелка у схвата показывает его поворот в XY.\n\nСтроки таблицы следуют по порядку. Время между строками одинаковое. В контурном режиме пары интерполируются, в позиционном задание переключается ступенчато.\n\nПри смене формата таблицы проверьте единицы или нажмите «Пример».')
        note.setWordWrap(True)
        note.setStyleSheet('color: #62768b; padding: 10px;')
        self.form_layout.addWidget(note)
        self.form_layout.addStretch()

    def set_visible(self,form,key,on):
        w=self.controls[key]
        w.setVisible(on)
        form.labelForField(w).setVisible(on)

    def update_fields(self):
        if not hasattr(self,'point_table'):
            return
        kind=self.controls['kind'].currentData()
        for k in ('l1','l2','l3','q2min','q2max'):
            self.set_visible(self.geometry,k,kind=='scara')
        for k in ('q1min','q1max'):
            self.set_visible(self.geometry,k,kind!='cartesian')
        for k in ('rmin','rmax'):
            self.set_visible(self.geometry,k,kind=='cylindrical')
        for k in ('xmax','ymax'):
            self.set_visible(self.geometry,k,kind=='cartesian')
        path=self.controls['path'].currentData()
        contour=self.controls['mode'].currentData()=='contour'
        self.controls['path'].model().item(2).setEnabled(contour)
        if path=='circle' and not contour:
            self.controls['path'].setCurrentIndex(1)
            path='line'
        circle=path=='circle'
        table=path=='table'
        self.table_group.setVisible(table)
        self.set_visible(self.motion,'coordinate_mode',table)
        for k in ('center_x','center_y','radius'):
            self.set_visible(self.motion,k,circle)
        for k in ('end_x','end_y','start_x','start_y'):
            self.set_visible(self.motion,k,not circle and not table)
        joint=self.controls['coordinate_mode'].currentData()=='joint'
        headers=(['q₁, °','q₂, °'] if kind=='scara' else
                 ['q₁, °','Радиус, мм'] if kind=='cylindrical' else ['q₁, мм','q₂, мм'])
        self.point_table.setHorizontalHeaderLabels(headers if joint else ['X, мм','Y, мм'])

    def populate(self,c):
        for key,w in self.controls.items():
            w.blockSignals(True)
            if isinstance(w,QComboBox):
                w.setCurrentIndex(w.findData(getattr(c,key)))
            else:
                w.setValue(getattr(c,key))
            w.blockSignals(False)
        self.set_points(c.points)
        self.update_fields()

    def set_points(self,points):
        self.point_table.blockSignals(True)
        self.point_table.setRowCount(len(points))
        for row,pair in enumerate(points):
            for col,v in enumerate(pair):
                self.point_table.setItem(row,col,QTableWidgetItem(f'{v:g}'))
        self.point_table.blockSignals(False)

    def add_point(self):
        if self.point_table.rowCount()>=200:
            self.failure(ValueError('Не более 200 строк.'))
            return
        row=self.point_table.rowCount()
        pair=[self.point_table.item(row-1,col).text() if row and self.point_table.item(row-1,col) else '0' for col in range(2)]
        self.point_table.insertRow(row)
        for col,v in enumerate(pair):
            self.point_table.setItem(row,col,QTableWidgetItem(v))
        self.invalidate()

    def remove_point(self):
        if self.point_table.rowCount()<=2:
            self.failure(ValueError('Оставьте минимум две строки.'))
            return
        rows={i.row() for i in self.point_table.selectedIndexes()}
        if not rows:
            rows={self.point_table.rowCount()-1}
        for row in sorted(rows,reverse=True):
            if self.point_table.rowCount()>2:
                self.point_table.removeRow(row)
        self.invalidate()

    def example_points(self):
        if self.controls['coordinate_mode'].currentData()=='cartesian':
            points=Config().points
        else:
            kind=self.controls['kind'].currentData()
            points=([[-30,80],[-10,110],[10,80],[30,110],[50,80]] if kind=='scara' else
                    [[-40,300],[-20,450],[0,300],[20,450],[40,300]] if kind=='cylindrical' else Config().points)
        self.set_points(points)
        self.invalidate()

    def read_config(self):
        values={k:(w.currentData() if isinstance(w,QComboBox) else w.value()) for k,w in self.controls.items()}
        points=[]
        for row in range(self.point_table.rowCount()):
            try:
                pair=[float(self.point_table.item(row,col).text().strip().replace(',','.')) for col in range(2)]
            except (AttributeError,ValueError):
                raise ValueError(f'Строка {row+1}: введите два числа.') from None
            points.append(pair)
        values['points']=points
        return Config(**values).validate()

    def stop_animation(self):
        self.timer.stop()
        self.play.setText('▶ Анимация')

    def invalidate(self):
        self.stop_animation()
        self.result=None
        self.play.setEnabled(False)
        self.export_button.setEnabled(False)
        for metric in self.metrics:
            metric.setText('—')
        self.progress.setValue(0)
        self.status.setText('Параметры изменены. Графики предыдущего расчёта устарели — пересчитайте.')
        for plot in (self.workspace_plot,self.path_plot,self.error_plot,self.signals_plot):
            if plot.ax.has_data():
                plot.ax.set_title('Предыдущие параметры — требуется пересчёт',fontsize=10,color='#b06a20')
                plot.refresh()

    def failure(self,exc):
        self.status.setText(str(exc))
        QMessageBox.warning(self,'Проверьте параметры',str(exc))

    def axes_xy(self,plot,title):
        ax=plot.ax
        ax.clear()
        ax.set_title(title,loc='left',fontsize=12,pad=15)
        ax.set_xlabel('X, мм')
        ax.set_ylabel('Y, мм')
        ax.set_aspect('equal',adjustable='box')
        ax.grid(alpha=.18)
        return ax

    def equal_axes(self,ax,points):
        import numpy as np
        p=np.array(points)[:,:2]
        lo=p.min(axis=0)
        hi=p.max(axis=0)
        center=(lo+hi)/2
        span=max(max(hi-lo),1)*.55
        ax.set_xlim(center[0]-span,center[0]+span)
        ax.set_ylim(center[1]-span,center[1]+span)
        ax.set_aspect('equal',adjustable='box')

    def xy(self,points):
        return ([p[0] for p in points],[p[1] for p in points])

    def heading_points(self,c,sample):
        x,y,_=sample.actual
        phi=tool_angle(c,sample.q)
        length=max(c.l1+c.l2+c.l3 if c.kind=='scara' else c.rmax if c.kind=='cylindrical' else max(c.xmax,c.ymax),1)*.08
        return (x,y),(x+length*math.cos(phi),y+length*math.sin(phi))

    def robot_line(self,c,q):
        from math import sin,cos
        a,b,d,z=q
        if c.kind=='scara':
            return [(0,0,0),(0,0,z),(c.l1*cos(a),c.l1*sin(a),z),
                    (c.l1*cos(a)+c.l2*cos(a+b),c.l1*sin(a)+c.l2*sin(a+b),z),forward(c,q)]
        if c.kind=='cylindrical':
            return [(0,0,0),(0,0,z),forward(c,q)]
        return [(0,0,0),(a,0,0),(a,b,0),forward(c,q)]

    def build_workspace(self):
        try:
            c=self.read_config()
            pts=workspace(c)[::2]
            ax=self.axes_xy(self.workspace_plot,f'{NAMES[c.kind]} · рабочая область XY, вид сверху')
            ax.scatter(*self.xy(pts),s=2,alpha=.16,color='#13a99b',rasterized=True)
            q=(joint_pair(c,c.points[0]) if c.path=='table' and c.coordinate_mode=='joint' else inverse(c,desired(c,0)))
            if q:
                ax.plot(*self.xy(self.robot_line(c,q)),color='#183b59',linewidth=5,marker='o',markersize=6)
            self.equal_axes(ax,pts)
            self.workspace_plot.refresh()
            self.tabs.setCurrentIndex(0)
            self.status.setText(f'Вид сверху XY, Z = {c.start_z:g} мм. Область для всех ориентаций схвата; граница приближённая.')
        except ValueError as e:
            self.failure(e)

    def run_simulation(self):
        self.stop_animation()
        try:
            c=self.read_config()
            result=simulate(c)
            self.result=result
            self.config=c
            ax=self.axes_xy(self.path_plot,f'Горизонтальная плоскость XY · Z = {c.start_z:g} мм')
            samples=result.samples
            ax.plot(*self.xy([s.desired for s in samples]),color='#13a99b',linestyle='--',linewidth=2,label='Заданный след')
            self.trace_artist,=ax.plot(*self.xy([s.actual for s in samples]),color='#274d76',linewidth=2,label='След схвата')
            ax.legend(loc='upper left',fontsize=9)
            line=self.robot_line(c,samples[-1].q)
            self.robot_artist,=ax.plot(*self.xy(line),color='#e29940',marker='o',linewidth=4)
            self.heading_artist=FancyArrowPatch(*self.heading_points(c,samples[-1]),arrowstyle='-|>',mutation_scale=16,color='#b44d66',linewidth=2)
            ax.add_patch(self.heading_artist)
            bounds=[s.desired for s in samples]+[s.actual for s in samples]
            # Include joint locations throughout the animation, not only at the last pose.
            for s in samples[::max(1,len(samples)//200)]:
                bounds.extend(self.robot_line(c,s.q))
                bounds.extend((*p,c.start_z) for p in self.heading_points(c,s))
            bounds.extend((*p,c.start_z) for p in self.heading_points(c,samples[-1]))
            self.equal_axes(ax,bounds)
            self.path_plot.refresh()
            ax=self.error_plot.ax
            ax.clear()
            times=[s.t for s in samples]
            errors=[s.error for s in samples]
            ax.plot(times,errors,color='#087f75',linewidth=2)
            ax.fill_between(times,errors,color='#13a99b',alpha=.12)
            ax.set_xlabel('Время, с')
            ax.set_ylabel('Позиционная ошибка, мм')
            ax.set_title('Евклидова ошибка положения во времени',loc='left',fontsize=12)
            ax.grid(alpha=.18)
            self.error_plot.refresh()
            ax=self.signals_plot.ax
            ax.clear()
            for j,name,color in [(0,'X','#274d76'),(1,'Y','#13a99b')]:
                ax.plot(times,[s.actual[j] for s in samples],color=color,label=f'{name}(t) фактическая')
                ax.plot(times,[s.desired[j] for s in samples],color=color,linestyle='--',alpha=.5,label=f'{name}(t) заданная')
            ax.set_xlabel('Время, с')
            ax.set_ylabel('Координата, мм')
            ax.set_title('Осциллограммы горизонтальных координат',loc='left',fontsize=12)
            ax.grid(alpha=.18)
            ax.legend(fontsize=9)
            self.signals_plot.refresh()
            for w,v in zip(self.metrics,(result.maximum,result.rms,result.final)):
                w.setText(f'{v:.4f}')
            self.play.setEnabled(True)
            self.export_button.setEnabled(True)
            self.animation_index=0
            self.progress.setValue(0)
            self.tabs.setCurrentIndex(1)
            self.status.setText(f'Расчёт завершён: {len(samples)} отсчётов. Учебная модель; СКО включает переходный процесс.')
        except ValueError as e:
            self.invalidate()
            self.failure(e)

    def toggle_animation(self):
        if self.timer.isActive():
            self.stop_animation()
            return
        if not self.result:
            return
        if self.animation_index>=len(self.result.samples):
            self.animation_index=0
        if self.animation_index==0:
            self.draw_frame(0)
        self.tabs.setCurrentIndex(1)
        self.play.setText('Ⅱ Пауза')
        self.timer.start()

    def draw_frame(self,i):
        sample=self.result.samples[i]
        self.robot_artist.set_data(*self.xy(self.robot_line(self.result.config,sample.q)))
        self.trace_artist.set_data(*self.xy([s.actual for s in self.result.samples[:i+1]]))
        self.heading_artist.set_positions(*self.heading_points(self.result.config,sample))
        self.path_plot.refresh()
        self.progress.setValue(round(100*i/(len(self.result.samples)-1)))
        phi=math.degrees(tool_angle(self.result.config,sample.q))
        self.status.setText(f'Время {sample.t:.2f} с · X={sample.actual[0]:.2f}, Y={sample.actual[1]:.2f} мм · φ={phi:.2f}° · ошибка {sample.error:.4f} мм')

    def animate(self):
        if not self.result:
            return
        i=min(self.animation_index,len(self.result.samples)-1)
        self.draw_frame(i)
        self.animation_index+=max(1,round(.04/self.result.config.dt))
        if i==len(self.result.samples)-1:
            self.stop_animation()

    def save_project(self):
        try:
            c=self.read_config()
            path,_=QFileDialog.getSaveFileName(self,'Сохранить проект','robot-project.json','Проект ROBOT Lab (*.json)')
            if path:
                c.save(path)
                self.status.setText(f'Проект сохранён: {path}')
        except (OSError,ValueError) as e:
            self.failure(e)

    def load_project(self):
        path,_=QFileDialog.getOpenFileName(self,'Открыть проект','','Проект ROBOT Lab (*.json)')
        if not path:
            return
        try:
            c=Config.load(path)
            self.invalidate()
            self.populate(c)
            self.config=c
            self.build_workspace()
            self.status.setText('Проект загружен. Выполните расчёт движения.')
        except (OSError,ValueError,TypeError) as e:
            self.failure(e)

    def export_csv(self):
        if not self.result:
            return
        path,_=QFileDialog.getSaveFileName(self,'Экспорт результатов','robot-results.csv','CSV (*.csv)')
        if path:
            try:
                self.result.export_csv(path)
                self.status.setText(f'Результаты сохранены: {path}')
            except OSError as e:
                self.failure(e)

    def help(self):
        text=(Path(__file__).resolve().parent.parent/'docs'/'MODEL.md').read_text(encoding='utf-8')
        dialog=QMessageBox(self)
        dialog.setWindowTitle('Методика и ограничения')
        dialog.setText('Новая учебная модель ROBOT Lab. Не является точной копией DOS-решателя.')
        dialog.setDetailedText(text)
        dialog.exec()

    def closeEvent(self,event):
        self.stop_animation()
        super().closeEvent(event)


def main():
    app=QApplication(sys.argv)
    app.setStyle('Fusion')
    app.setStyleSheet(STYLE)
    window=Window()
    window.show()
    return app.exec()
