This file will just be for explaining what I have learnt in my short time using KiCad, that may help if it is your first time. I will try and give a list of things that are good to know when editing the schematics and some shortcuts etc.

1. Netclasses:
One of the more useful features of KiCad is netclasses. This is essentially a way of saying "Anything with this name is part of this net". This is useful for two reasons. The first is that anything with an exact net name will be electrically connected (useful for global power symbols such as the ones that go between the ADS power circuit and the compute and actuator drive circuits). The second is a little bit more of an aesthetic reason. It is possible to assign a netclass a wire thickness, color, type and then possible to assign any name to that netclass. For example I can create the netclass "SDC_SPINE", make its wire thickness 0.3 mm, its colour dark red, and its linetype solid. I then can create a netclass assignment with the pattern "SDC_SPINE*" and anything with that label or connected to a net starting with that string, will become that colour, thickness and linetype.

2. Symbol Editing:
It is useful to know that a single symbol can have multiple components that can be spread out across a schematic (amount of units). That the convention for pin spacing on a symbol is 100 mm. That the grid size when editing symbols (and in general) can be adjusted for more accurate drawing of the symbol. That new symbols can be made using parts or all of old ones. That pin names do not have to be shown, and if they are that they do not have to be shown inside the symbol body. That there is a pre-existing library called "FSUK_ADS" that symbols should be taken out of and added into. This will be in the files and have the termination ".sym".

3. Shortcuts:
A - Add a symbol
T - Add text
G - If pressed after selecting a component, will move the component without breaking the wired connections
M - Will move the component, disregarding the wired connections
P - Place power symbols
W - Use the wire tool
Q - Add a no connect symbol
L - Label something
S - Draw a hierarchical sheet

Creating a box by clicking and dragging with your mouse - If you drag from top left to bottom right, it only selects things completely enclosed in the box. If you drag from bottom right to top left, it selects everything even touched by the box.

There are way more but you will just figure them out as you go. For people in future who used this file to help them learn, please update it as you go along in order to help future people :)
