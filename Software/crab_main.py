#!/usr/bin/env python

# The MIT License (MIT)
#
# Copyright (c) 2015 Paul Wachendorf <paul.wachendorf@web.de>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""Steam Controller Callbacks"""
import sys
from crab_callbacks import crabBot
from subsections import body, hexleg, robotarm

from steamcontroller import SteamController, SCButtons
from steamcontroller.events import EventMapper, Pos

crab = crabBot()

def steam_pressed(evm, btn, pressed):
    crab.steam_pressed()

def a_pressed(evm, btn, pressed):
    crab.a_pressed()

def b_pressed(evm, btn, pressed):
    crab.b_pressed()

def x_pressed(evm, btn, pressed):
    crab.x_pressed()

def y_pressed(evm, btn, pressed):
    crab.y_pressed()

def bumper_pressed(evm, btn, pressed):
    crab.bumper_pressed()

def grip_pressed(evm, btn, pressed):
    crab.grip_pressed()

def start_pressed(evm, btn, pressed):
    crab.start_pressed()

def back_pressed(evm, btn, pressed):
    crab.back_pressed()

def pad_axes(evm, pad, dx, dy):
    crab.pad_axes(pad, dx, dy)

def pad_pressed(evm, pad, dTheta):
    crab.pad_pressed(pad, dTheta)

def stick_axes(evm, x, y):
    crab.stick_axes(x, y)

def stick_pressed(evm, btn, pressed):
    crab.stick_pressed(btn, pressed)

def trig_axes(evm, pos, value):
    crab.trig_axes(pos, value)

def evminit():
    evm = EventMapper()
    evm.setButtonCallback(SCButtons.STEAM, steam_pressed)
    evm.setButtonCallback(SCButtons.A,a_pressed)
    evm.setButtonCallback(SCButtons.B, b_pressed)
    evm.setButtonCallback(SCButtons.X, x_pressed)
    evm.setButtonCallback(SCButtons.Y, y_pressed)
    evm.setButtonCallback(SCButtons.LB, bumper_pressed)
    evm.setButtonCallback(SCButtons.RB, bumper_pressed)
    evm.setButtonCallback(SCButtons.LGRIP, grip_pressed)
    evm.setButtonCallback(SCButtons.RGRIP, grip_pressed)
    evm.setButtonCallback(SCButtons.START, start_pressed)
    evm.setButtonCallback(SCButtons.BACK, back_pressed)
    evm.setPadButtonCallback(Pos.LEFT, pad_axes)
    evm.setPadButtonCallback(Pos.RIGHT, pad_axes)
    evm.setPadButtonCallback(Pos.LEFT, pad_pressed, clicked=True)
    evm.setPadButtonCallback(Pos.RIGHT, pad_pressed, clicked=True)
    evm.setStickAxesCallback(stick_axes)
    evm.setStickPressedCallback(stick_pressed)
    evm.setTrigAxesCallback(Pos.RIGHT, trig_axes)
    evm.setTrigAxesCallback(Pos.LEFT, trig_axes)
    return evm


if __name__ == '__main__':
    evm = evminit()
    sc = SteamController(callback=evm.process)
    sc.run()


