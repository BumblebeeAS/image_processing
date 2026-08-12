import itertools
from typing import Iterable, Optional, Sequence

import numpy as np
from foxglove_msgs.msg import (
    Color,
    ImageAnnotations,
    Point2,
    PointsAnnotation,
    TextAnnotation,
)
from std_msgs.msg import Header

# Source: https://supervision.roboflow.com/draw/color/#supervision.draw.color.ColorPalette.DEFAULT
DEFAULT_COLOR_PALETTE = [
    "#e6194b",
    "#3cb44b",
    "#ffe119",
    "#0082c8",
    "#f58231",
    "#911eb4",
    "#46f0f0",
    "#f032e6",
    "#d2f53c",
    "#fabebe",
    "#008080",
    "#e6beff",
    "#aa6e28",
    "#fffac8",
    "#800000",
    "#aaffc3",
]


def get_image_annotations(
    header: Header,
    point_sets_list: Iterable[Iterable[np.ndarray]],
    colors: Sequence[str] = DEFAULT_COLOR_PALETTE,
    points_annotation_type: int = PointsAnnotation.LINE_LOOP,
    thickness: float = 5.0,
    labels: Optional[Iterable[Iterable[str]]] = None,
    font_size: float = 20.0,
) -> ImageAnnotations:
    """Get points colored by their sublist's index, optionally with text labels.

    Args:
        header (Header): Header for the annotations.
        point_sets_list (Iterable[Iterable[np.ndarray]]): List of lists of point sets, where
            each sublist of point sets is colored by their index in the list of lists. A point
            set can be a collection of unordered points or polygons, etc.
        colors (Sequence[str]): List of hex color strings to use for each sublist.
            Defaults to DEFAULT_COLOR_PALETTE.
        points_annotation_type (int): (POINTS, LINE_LOOP, LINE_LIST, LINE_STRIP).
        thickness (int): PointsAnnotation thickness.
        labels (Optional[Iterable[Iterable[str]]]): Text label per point set, in the
            same nested shape as ``point_sets_list`` (``labels[i][j]`` labels
            ``point_sets_list[i][j]``), mirroring how ``colors`` is aligned to the
            sublists. Each non-empty label becomes a TextAnnotation anchored at its
            point set's top-most vertex, using the sublist color as the background.
            When None (default) no text is emitted -- behavior matches callers that
            only draw outlines.
        font_size (float): TextAnnotation font size in pixels.

    Returns:
        ImageAnnotations: An ImageAnnotations message containing PointsAnnotations
            for each polygon, plus a TextAnnotation for each provided label.
    """

    def hex_to_rgba(hex_color: str) -> tuple:
        hex_color = hex_color.lstrip("#")
        r, g, b = (
            int(hex_color[0:2], 16),
            int(hex_color[2:4], 16),
            int(hex_color[4:6], 16),
        )
        a = 255
        return (r, g, b, a)

    def get_color(i: int) -> Color:
        r, g, b, a = (c / 255.0 for c in hex_to_rgba(colors[i % len(colors)]))
        return Color(r=r, g=g, b=b, a=a)

    def get_annotation(color: Color, point_set) -> PointsAnnotation:
        points = [Point2(x=float(x), y=float(y)) for x, y in point_set]
        annotation = PointsAnnotation(
            timestamp=header.stamp,
            type=points_annotation_type,
            points=points,
            outline_color=color,
            thickness=thickness,
        )
        return annotation

    def get_text_annotation(color: Color, point_set, text: str) -> TextAnnotation:
        # Anchor the label at the point set's top-most vertex (smallest image y).
        pts = np.asarray(point_set, dtype=float).reshape(-1, 2)
        anchor = pts[int(np.argmin(pts[:, 1]))]
        return TextAnnotation(
            timestamp=header.stamp,
            position=Point2(x=float(anchor[0]), y=float(anchor[1])),
            text=text,
            font_size=font_size,
            text_color=Color(r=1.0, g=1.0, b=1.0, a=1.0),
            background_color=color,
        )

    point_annotations = []
    text_annotations = []

    # labels, when given, is aligned to point_sets_list group-for-group; when not,
    # repeat None so every group falls through to the outline-only path.
    label_groups = labels if labels is not None else itertools.repeat(None)
    for i, (point_sets, label_set) in enumerate(zip(point_sets_list, label_groups)):
        color = get_color(i)
        # zip_longest so a short/missing label list never drops a point set.
        if label_set is None:
            pairs = ((point_set, None) for point_set in point_sets)
        else:
            pairs = itertools.zip_longest(point_sets, label_set, fillvalue=None)
        for point_set, label in pairs:
            if point_set is None:
                continue
            point_annotations.append(get_annotation(color, point_set))
            if label:
                text_annotations.append(get_text_annotation(color, point_set, label))

    image_annotations = ImageAnnotations(
        points=point_annotations, texts=text_annotations
    )
    return image_annotations
