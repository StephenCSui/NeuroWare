"""Grid layout for the transport warehouse -- structurally the same pattern
as simulation/grid.py: a cols x rows Cell grid, base-filled with one type,
named regions stamped on top, BFS shortest_path for routing. Robot movement
itself stays continuous (see entities.Robot), the grid is bookkeeping only.
"""
import heapq

from config import (DROPOFF_REGION, GRID_CELL, GRID_COLS, GRID_ROWS,
                     SHELF_REGIONS, SLOT_SPAN, STAGING_REGION)


class Cell:
    def __init__(self, col, row, cell_type):
        self.col = col
        self.row = row
        self.cell_type = cell_type   # "walkway" | "wall" | "shelf" | "dropoff" | "staging"
        self.blocked = cell_type in ("wall", "shelf")

    def center(self):
        return (self.col + 0.5) * GRID_CELL, (self.row + 0.5) * GRID_CELL


class ShelfSlot:
    """A SLOT_SPAN-cell run of one shelf that holds at most one object at a
    time. Its cells are blocked by default, like every other shelf cell, and
    only open up while THIS slot specifically holds an object -- other
    slots on the same shelf, empty or otherwise, are never affected."""
    def __init__(self, cells):
        self.cells = cells
        self.item = None   # the TransportObject currently sitting here, or None

    def occupy(self, transport_object):
        self.item = transport_object
        for c in self.cells:
            c.blocked = False

    def clear(self):
        self.item = None
        for c in self.cells:
            c.blocked = True

    def center(self):
        cx = sum(c.col for c in self.cells) / len(self.cells)
        cy = sum(c.row for c in self.cells) / len(self.cells)
        return (cx + 0.5) * GRID_CELL, (cy + 0.5) * GRID_CELL

    def world_bounds(self):
        cols = [c.col for c in self.cells]
        rows = [c.row for c in self.cells]
        x0, y0 = min(cols) * GRID_CELL, min(rows) * GRID_CELL
        x1, y1 = (max(cols) + 1) * GRID_CELL, (max(rows) + 1) * GRID_CELL
        return x0, y0, x1, y1


class Warehouse:
    def __init__(self):
        self.cols = GRID_COLS
        self.rows = GRID_ROWS
        self.cells = self._build_layout()
        self.slots = self._build_slots()

    def _build_layout(self):
        cells = [[Cell(col, row, "walkway") for row in range(self.rows)]
                 for col in range(self.cols)]

        for col in range(self.cols):
            cells[col][0] = Cell(col, 0, "wall")
            cells[col][self.rows - 1] = Cell(col, self.rows - 1, "wall")
        for row in range(self.rows):
            cells[0][row] = Cell(0, row, "wall")
            cells[self.cols - 1][row] = Cell(self.cols - 1, row, "wall")

        def stamp(region, cell_type):
            col0, row0, w, h = region
            for col in range(col0, col0 + w):
                for row in range(row0, row0 + h):
                    if 0 <= col < self.cols and 0 <= row < self.rows:
                        cells[col][row] = Cell(col, row, cell_type)

        for region in SHELF_REGIONS:
            stamp(region, "shelf")
        stamp(DROPOFF_REGION, "dropoff")
        stamp(STAGING_REGION, "staging")

        return cells

    def _build_slots(self):
        slots = []
        for region in SHELF_REGIONS:
            col0, row0, w, h = region
            horizontal = w >= h
            length = w if horizontal else h
            for start in range(0, length, SLOT_SPAN):
                span = min(SLOT_SPAN, length - start)
                if horizontal:
                    cells = [self.cells[col0 + start + i][row0] for i in range(span)]
                else:
                    cells = [self.cells[col0][row0 + start + i] for i in range(span)]
                slots.append(ShelfSlot(cells))
        return slots

    # ------------------------------------------------------------------
    # Lookup / topology -- ported from simulation/grid.py
    # ------------------------------------------------------------------

    def cell_at(self, col, row):
        if 0 <= col < self.cols and 0 <= row < self.rows:
            return self.cells[col][row]
        return None

    def world_to_cell(self, x, y):
        return int(x // GRID_CELL), int(y // GRID_CELL)

    def neighbors(self, cell):
        out = []
        for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            n = self.cell_at(cell.col + dc, cell.row + dr)
            if n is not None:
                out.append(n)
        return out

    def shortest_path(self, start, target, blocked_fn=None):
        """Dijkstra over unblocked cells, uniform per-cell cost -- same
        approach as simulation/grid.py's shortest_path, minus the
        avoid_occupied fallback (robots here aren't grid-exclusive, so the
        swap-deadlock case it existed for doesn't apply the same way).

        blocked_fn overrides the default single-point cell.blocked check --
        used by route_for_footprint() to validate a whole object's bounding
        box at each candidate cell, not just the cell itself, so a wide
        object can't be routed through a passage only a point would fit."""
        if blocked_fn is None:
            blocked_fn = lambda c: c.blocked
        if start is target:
            return [start]
        dist    = {start: 0}
        parent  = {}
        visited = set()
        heap    = [(0, id(start), start)]
        while heap:
            d, _, cur = heapq.heappop(heap)
            if cur in visited:
                continue
            visited.add(cur)
            if cur is target:
                break
            for n in self.neighbors(cur):
                if blocked_fn(n) or n in visited:
                    continue
                nd = d + 1
                if n not in dist or nd < dist[n]:
                    dist[n]   = nd
                    parent[n] = cur
                    heapq.heappush(heap, (nd, id(n), n))
        if target not in visited:
            return []
        path = [target]
        while path[-1] is not start:
            path.append(parent[path[-1]])
        path.reverse()
        return path

    def escape_point(self, slot):
        """First safe move for an object leaving this slot. BFS routing
        (route(), shortest_path()) treats the object as a single point, but
        a shelf object is SLOT_SPAN cells wide -- shifting it sideways along
        its own row/col to the "next cell over" pokes its far edge into a
        neighboring, unrelated (and likely blocked) slot, even though that
        next cell itself is open. Moving straight off the shelf
        perpendicular to its orientation doesn't have that problem (the
        object's width stays within its own already-open slot the whole
        way), so that's always the correct first move, before any BFS
        routing along open floor takes over."""
        mid = slot.cells[len(slot.cells) // 2]
        horizontal = slot.cells[0].row == slot.cells[-1].row
        if horizontal:
            candidates = [self.cell_at(mid.col, mid.row - 1), self.cell_at(mid.col, mid.row + 1)]
        else:
            candidates = [self.cell_at(mid.col - 1, mid.row), self.cell_at(mid.col + 1, mid.row)]
        for cand in candidates:
            if cand is not None and not cand.blocked:
                return cand.center()
        return slot.center()   # shouldn't normally happen given this layout

    def route(self, x0, y0, x1, y1):
        """World-space waypoint path from (x0,y0) to (x1,y1), via BFS cell
        centers with the final waypoint snapped to the exact target (the
        target is rarely a cell center -- e.g. an underneath-lift point
        offset within a multi-cell slot). Point-sized routing -- for robots,
        not the wider transport objects (see route_for_footprint)."""
        start  = self.cell_at(*self.world_to_cell(x0, y0))
        target = self.cell_at(*self.world_to_cell(x1, y1))
        if start is None or target is None:
            return []
        path_cells = self.shortest_path(start, target)
        if not path_cells:
            return []
        waypoints = [c.center() for c in path_cells[1:]]   # skip robot's current cell
        if waypoints:
            waypoints[-1] = (x1, y1)
        else:
            waypoints = [(x1, y1)]
        return waypoints

    def route_for_footprint(self, x0, y0, x1, y1, width, height):
        """Same idea as route(), but for an object with real width/height --
        a candidate cell only counts as passable if the object's actual
        bounding box, centered there, doesn't overlap any obstacle. This is
        what keeps a wide object from being routed through a gap only its
        center point would clear (e.g. squeezing past the vertical shelf's
        edge without a real margin)."""
        start  = self.cell_at(*self.world_to_cell(x0, y0))
        target = self.cell_at(*self.world_to_cell(x1, y1))
        if start is None or target is None:
            return []

        def footprint_blocked(cell):
            cx, cy = cell.center()
            bx0, by0 = cx - width / 2, cy - height / 2
            bx1, by1 = cx + width / 2, cy + height / 2
            return self.rect_overlaps_obstacle(bx0, by0, bx1, by1)

        path_cells = self.shortest_path(start, target, blocked_fn=footprint_blocked)
        if not path_cells:
            return []
        waypoints = [c.center() for c in path_cells[1:]]
        if waypoints:
            waypoints[-1] = (x1, y1)
        else:
            waypoints = [(x1, y1)]
        return waypoints

    # ------------------------------------------------------------------
    # Obstacle queries -- used by negotiation.py's local repulsion sensing
    # and sim.py's hard collision check during the transport phase
    # ------------------------------------------------------------------

    def is_blocked_cell(self, col, row):
        cell = self.cell_at(col, row)
        return cell is None or cell.blocked

    def obstacle_cells_near(self, x, y, radius):
        c0, r0 = self.world_to_cell(x, y)
        span = int(radius // GRID_CELL) + 1
        found = []
        for r in range(r0 - span, r0 + span + 1):
            for c in range(c0 - span, c0 + span + 1):
                if self.is_blocked_cell(c, r):
                    cx, cy = (c + 0.5) * GRID_CELL, (r + 0.5) * GRID_CELL
                    if (cx - x) ** 2 + (cy - y) ** 2 <= radius ** 2:
                        found.append((cx, cy))
        return found

    def rect_overlaps_obstacle(self, x0, y0, x1, y1):
        # x1/y1 are exclusive rect edges that, for grid-aligned footprints
        # (every object/slot here is), land exactly on a cell boundary --
        # plain floor division would round that up to the NEXT cell, so an
        # object's own bounds could spuriously overlap a neighboring,
        # unrelated slot. Nudge the far edge back a hair before converting.
        c0, r0 = self.world_to_cell(x0, y0)
        c1, r1 = self.world_to_cell(x1 - 1e-6, y1 - 1e-6)
        for r in range(r0, r1 + 1):
            for c in range(c0, c1 + 1):
                if self.is_blocked_cell(c, r):
                    return True
        return False

    # ------------------------------------------------------------------
    # Named zone bounds, in world px
    # ------------------------------------------------------------------

    def staging_bounds(self):
        return self._region_bounds(STAGING_REGION)

    def dropoff_bounds(self):
        return self._region_bounds(DROPOFF_REGION)

    @staticmethod
    def _region_bounds(region):
        col, row, w, h = region
        return col * GRID_CELL, row * GRID_CELL, (col + w) * GRID_CELL, (row + h) * GRID_CELL
