import os
cols, rows = os.get_terminal_size()


class Menu():
    def __init__(self):
        self.title = "Example Title"
        self.intro = None
        self.rows = 20
        self.cols = 60
        self.padding = 2
        self.borderSide = '│'
        self.BorderTopBot = '─'
        self.borderTopR = '┐'
        self.borderTopL = '┌'
        self.borderBotR = '┘'
        self.borderBotL = '└'

    def SetBorder(self, topRight='┐', topLeft='┌', bottomRight='┘', bottomLeft='└',side='│'):
        self.borderSide = side
        self.borderTopR = topRight
        self.borderTopL = topLeft
        self.borderBotR = bottomRight
        self.borderBotL = bottomLeft

    def SetIntro(self, intro):
        self.intro = str(intro)

    def SetPadding(self, padAmount):
        self.padding = padAmount

    def SetTitle(self, title):
        self.title = title

    def SetSize(self, rows, cols):
        self.rows = rows
        self.cols = cols

    def AdjustWidth(self):
        self.cols = int(self.cols - (self.padding * 2))

    def CreateMenu(self):
        padding = (" " * (self.padding))
        topBorder = int(((((self.cols-2) - len(self.title))) / 2) -1) * self.BorderTopBot
        # Draw top border
        fullMenu = [padding + self.borderTopL + (topBorder) + " " + self.title + " " + (topBorder) + self.borderTopR]
        # Introductionary text - padd first row
        emptyRow = padding + self.borderSide + (' ' * (self.cols-2)) + self.borderSide
        fullMenu.append(emptyRow)
        if self.intro:
            introText = padding + self.borderSide + " " + self.intro + (' ' * (self.cols - len(self.intro) - 3)) + self.borderSide
            fullMenu.append(introText)
        # Draw side borders and populate
        for i in range(1, self.rows):
            fullMenu.append(emptyRow)
        # Draw bottom border
        fullMenu.append(padding + self.borderBotL + (self.BorderTopBot * (self.cols-2)) + self.borderBotR)
        return '\n'.join(fullMenu)

    def DrawMenu(self):
        print(self.CreateMenu())


newMenu = Menu()
newMenu.SetIntro("This is my introduction test")
newMenu.DrawMenu()
