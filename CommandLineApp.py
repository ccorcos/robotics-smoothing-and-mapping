# from CommandLineApp import *
# terminal = CommandLineApp()
# if __name__ == "__main__":
#     terminal.start(main)

import curses
import signal
import sys
import traceback


class CommandLineApp:

    def __init__(self):
        self.window = curses.initscr()  # initialize key capture
        curses.cbreak()  # dont wait for <enter>
        self.window.erase()
        self.window.refresh()

        def ctrlC(signal, frame):
            self.doneCurses()
            sys.exit(0)

        # handle control-c to reset window
        signal.signal(signal.SIGINT, ctrlC)

    def doneCurses(self):
        curses.echo()
        curses.endwin()

    def start(self, method):
        try:
            method()
            self.doneCurses()
        except Exception, e:
            self.doneCurses()
            print traceback.print_tb(sys.exc_info()[2])
            print str(e)

    def noecho(self):
        curses.noecho()

    def echo(self):
        curses.echo()

    def nextLine(self):
        y, x = self.window.getyx()
        self.window.move(y + 1, 0)

    def println(self, string):
        y, x = self.window.getyx()
        self.window.addstr(y, 0, str(string))
        self.nextLine()

    def reprint(self, string):
        y, x = self.window.getyx()
        self.window.move(y, 0)
        self.window.clrtoeol()
        self.window.addstr(y, 0, str(string))

    def clearln(self):
        y, x = self.window.getyx()
        self.window.move(y, 0)
        self.window.clrtoeol()

    def moveUp(self):
        y, x = self.window.getyx()
        self.window.move(y - 1, 0)

    def moveDown(self):
        y, x = self.window.getyx()
        self.window.move(y + 1, 0)

    def clearUp(self, n):
        self.clearln()
        for i in range(n - 1):
            self.moveUp()
            self.clearln()

    def clearUpTo(self, y):
        self.clearln()
        yi, x = self.window.getyx()
        while yi > y:
            self.moveUp()
            self.clearln()
            yi = yi - 1
            self.window.move(yi, 0)

    def debugPrint(self, a, s=''):
        y, x = self.window.getyx()
        self.nextLine()
        self.println("debug: " + s)
        self.println(str(a))
        self.println("<enter> to continue")
        self.nextLine()
        while True:
            key = self.window.getch()    # wait for a key press
            if key == 27 or key == 10:  # escape to finish
                break
        self.clearUpTo(y)

    def wait(self):
        self.println("<enter> to continue...")
        while True:
            key = self.window.getch()    # wait for a key press
            if key == 27 or key == 10:  # escape to finish
                self.clearUp(2)
                break

    def waitMsg(self, msg):
        y, x = self.window.getyx()
        self.println(msg)
        self.println("<enter> to continue...")
        while True:
            key = self.window.getch()    # wait for a key press
            if key == 27 or key == 10:  # escape to finish
                self.clearUpTo(y)
                break

    def yesno(self, a):
        self.println(str(a))
        self.reprint("[y/n]:")
        while True:
            key = self.window.getch()    # wait for a key press
            if key == ord('y'):
                self.clearUp(2)
                return True
            elif key == ord('n'):
                self.clearUp(2)
                return False
            else:
                self.reprint("[y/n]:")

    def queryForString(self, string):
        y, x = self.window.getyx()
        self.window.addstr(y, 0, string)
        s = self.window.getstr()
        self.clearUpTo(y)
        return s

    def keyPress(self):
        return self.window.getch()
