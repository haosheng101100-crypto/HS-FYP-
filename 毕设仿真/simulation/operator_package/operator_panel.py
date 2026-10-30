"""Native GTK buttons; no flight action is sent merely by opening this panel."""
import os,json,time,uuid
from pathlib import Path
import gi
gi.require_version('Gtk','3.0')
from gi.repository import Gtk,GLib,Gdk
SESSION=Path(os.environ['HS_OPERATOR_SESSION'])

class Panel(Gtk.Window):
    def __init__(self):
        super().__init__(title='新规划器 · 移动平台着陆')
        self.set_default_size(390,540);self.set_keep_above(True)
        screen=self.get_screen();self.move(max(0,screen.get_width()-390),80)
        box=Gtk.Box(orientation=Gtk.Orientation.VERTICAL,spacing=12);box.set_border_width(18);self.add(box)
        title=Gtk.Label();title.set_markup('<b><big>移动平台着陆 · 在线重规划</big></b>');box.pack_start(title,False,False,0)
        self.status=Gtk.Label(label='正在初始化，尚未起飞');self.status.set_line_wrap(True);box.pack_start(self.status,False,False,0)
        self.details=Gtk.Label(label='');box.pack_start(self.details,False,False,0)
        self.scenario=Gtk.ComboBoxText()
        for key,label in [('constant','匀速平台'),('varying','加速 / 制动平台'),('turning','缓慢转弯平台'),('maneuvering','变速并转弯')]:self.scenario.append(key,label)
        self.scenario.set_active_id('constant');self.scenario.connect('changed',self.choose_scenario)
        box.pack_start(self.scenario,False,False,0)
        self.planner=Gtk.Label(label='新规划器准备中');self.planner.set_line_wrap(True);box.pack_start(self.planner,False,False,0)
        grid=Gtk.Grid(column_spacing=12,row_spacing=12);box.pack_start(grid,True,True,0)
        self.buttons={}
        for action,label,x,y in [('takeoff','起飞',0,0),('land','降落到小车',1,0),('pause','暂停',0,1),('resume','继续',1,1),('reset','复位到起始状态',0,2)]:
            b=Gtk.Button(label=label);b.set_size_request(145,64);b.connect('clicked',lambda _,a=action:self.send(a))
            grid.attach(b,x,y,2 if action=='reset' else 1,1);self.buttons[action]=b;b.set_sensitive(False)
        hint=Gtk.Label(label='起飞后只悬停，等待你点降落。\n暂停会冻结物理与任务时间。');hint.set_line_wrap(True);box.pack_start(hint,False,False,0)
        css=Gtk.CssProvider();css.load_from_data(b'button {font-size:18px;} label {font-size:16px;}')
        Gtk.StyleContext.add_provider_for_screen(screen,css,Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION)
        self.connect('delete-event',self.close);GLib.timeout_add(250,self.update)
    def choose_scenario(self,widget):
        p=SESSION/'requests'/f'{time.time_ns()}_scenario.json'
        temp=p.with_suffix('.tmp');temp.write_text(json.dumps({'action':'scenario','value':widget.get_active_id(),'source':'native_user_panel'}));temp.replace(p)
    def send(self,action):
        name=f'{time.time_ns()}_{uuid.uuid4().hex}.json';p=SESSION/'requests'/name
        p.write_text(json.dumps({'action':action,'source':'native_user_panel','real_wall_unix_s':time.time()}))
        self.status.set_text({'takeoff':'已发送起飞指令','land':'已发送降落指令','pause':'正在暂停','resume':'正在继续','reset':'正在复位'}[action])
    def update(self):
        try:
            d=json.loads((SESSION/'status.json').read_text());self.status.set_text(d['message'])
            height=d.get('body_height_m');self.details.set_text(('已暂停' if d.get('paused') else '仿真运行中')+('  |  高度 %.2f m'%height if height is not None else ''))
            self.scenario.set_sensitive(d.get('stage') in ('READY','HOVER'))
            self.planner.set_text(d.get('planner_message','新规划器准备中'))
            for action,b in self.buttons.items():b.set_sensitive(bool(d.get('buttons',{}).get(action,False)))
        except (FileNotFoundError,json.JSONDecodeError):pass
        return True
    def close(self,*_):
        (SESSION/'close.request').write_text('User closed native operator panel\n');Gtk.main_quit();return False

panel=Panel();panel.show_all();Gtk.main()
