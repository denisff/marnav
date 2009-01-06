#!/usr/bin/env python
import pygtk
import gtk
import gtk.glade


def on_key_press(widget, event):
    keyname = gtk.gdk.keyval_name(event.keyval)
    if event.state & gtk.gdk.CONTROL_MASK:
       if keyname == "f":
           ui.toggle_fullscreen()
       if keyname == "t":
           ui.toggle_tabs()
       if keyname == "n":
           ui.rotate_page()


class UI(object):

    def toggle_fullscreen(self):
        if self._fullscreen:
            self.w1.unfullscreen()
        else:
            self.w1.fullscreen()        
        self._fullscreen = not self._fullscreen

    def toggle_tabs(self):
        self.nb.set_show_tabs(self._show_tabs)
        self._show_tabs = not self._show_tabs

    def rotate_page(self):
        self.nb.set_current_page((self.nb.get_current_page()+1) %3)

    def __init__(self):

        self._fullscreen = False
        self._show_tabs = False

        self.tree = gtk.glade.XML("avNav.glade")
        self.nb = self.tree.get_widget("notebook1")
        self.w1 = self.tree.get_widget("windowMain")
        self.tree.signal_autoconnect(self)
        self.w1.connect("delete_event", lambda w,e: gtk.main_quit())
        self.w1.connect('key_press_event', on_key_press)
        self.w1.show()


if __name__ == "__main__":

    ui = UI()
    gtk.main()
