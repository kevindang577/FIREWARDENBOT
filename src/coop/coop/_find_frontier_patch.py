def find_frontier(self):
    if self.pose is None:
        return None
    rx, ry, _ = self.pose
    rgx, rgy = self.world2grid(rx, ry)

    best = None
    best_d2 = 1e18
    # set min_cells = 0 so we can use the robot cell itself
    min_cells = 0

    for gy in range(self.grid_n):
        row = self.cover[gy]
        for gx in range(self.grid_n):
            if row[gx] != 1:
                continue
            if not self.has_unseen_neighbor(gx, gy):
                continue
            d2 = (gx - rgx)**2 + (gy - rgy)**2
            if d2 < min_cells * min_cells:
                continue
            if d2 < best_d2:
                best_d2 = d2
                best = (gx, gy)
    return best
