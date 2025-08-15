class Matter: # 以初赛夹爪底部为基准
    """物料尺寸参数类, 以初赛夹爪底部为基准"""
    
    def __init__(self, jawheight, matterheight, grabheight):
        self.jawheight = jawheight # 夹爪下降高度
        self.matterheight = matterheight # 物料高度
        self.grabheight = grabheight # 物料夹取位置高度

        self.tableHeight = 65.8 # 原料区转盘高度（初赛夹爪底部为基准，并非真实高度）
        self.placeHeight = 15.0 # 放置高度, 物料底部距离托盘的高度
        self.hoverHeight = 35.0 # 悬停高度, 夹爪底部距离物料顶部的高度
        self.trayHeight = 158.6 # 托盘绝对高度
        self.dimianHeight = -16.7

        # 托盘区点位参数
        self.tray_place = [-118.3, -123.7, self.trayHeight + self.placeHeight + self.grabheight + self.jawheight] # 放置到托盘的位置
        self.tray_hover = [-118.3, -123.7, self.trayHeight + self.matterheight + self.hoverHeight + self.jawheight] # 夹取前夹爪悬停的位置
        self.tray_grab = [-118.3, -123.7, self.trayHeight + self.grabheight + self.jawheight] # 从托盘夹取的位置

        # 原料区参数
        self.catch_height = self.tableHeight + self.grabheight + self.jawheight # 原料区物料抓取高度

        # 色环区
        self.sehuan_height = self.dimianHeight + self.grabheight + self.jawheight
        self.sehuanhover_height = self.dimianHeight + self.matterheight + self.hoverHeight + self.jawheight