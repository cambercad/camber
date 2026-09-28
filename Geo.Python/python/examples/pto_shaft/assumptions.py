"""Illustrative proportions for the PTO showcase; dimensions are millimetres."""

# Published DEMA 67991 dimensions: overall 800–1100, bell diameter 120,
# both sockets 1-3/8 inch / six splines, triangular telescoping profile.
# https://www.stabilo-fachmarkt.de/gelenkwelle-zapfwelle-85-110-cm-standard-1-3-8-zoll-6-zaehne_818_1657/
# Component dimensions below are photo-derived, not manufacturing specifications.
SPLINE_TEETH = 6
SPLINE_MAJOR_DIAMETER = 34.925
SPLINE_ROOT_DIAMETER = 29.4

# Cross spacing and component sections are inferred from the photographs.
# The two overall lengths above and the 120 mm bell diameter are published.
COLLAPSED_LENGTH = 800.0
EXTENDED_LENGTH = 1100.0
TELESCOPIC_TRAVEL = EXTENDED_LENGTH - COLLAPSED_LENGTH
CROSS_SPACING = COLLAPSED_LENGTH - 150.0
PROFILE_WALL = 2.0
PROFILE_ACROSS_CORNERS = 44.0
PROFILE_CONNECTION_OFFSET = 44.0
OUTER_PROFILE_LENGTH = 500.0
INNER_PROFILE_LENGTH = 500.0
INNER_PROFILE_ACROSS_CORNERS = 34.0  # ~0.9 mm clearance at the straight flanks
MIN_VISIBLE_ENGAGEMENT = (2 * PROFILE_CONNECTION_OFFSET + OUTER_PROFILE_LENGTH
                          + INNER_PROFILE_LENGTH - CROSS_SPACING - TELESCOPIC_TRAVEL)

assert SPLINE_ROOT_DIAMETER < SPLINE_MAJOR_DIAMETER
assert TELESCOPIC_TRAVEL > 0
assert MIN_VISIBLE_ENGAGEMENT > 0
