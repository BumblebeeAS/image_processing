from typing import Iterable, Sequence

import numpy as np
from foxglove_msgs.msg import Color, ImageAnnotations, Point2, PointsAnnotation
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
    thickness: int = 5,
) -> ImageAnnotations:
    """Get points colored by their sublist's index.

    Args:
        header (Header): Header for the annotations.
        point_sets_list (Iterable[Iterable[np.ndarray]]): List of lists of point sets, where
            each sublist of point sets is colored by their index in the list of lists. A point
            set can be a collection of unordered points or polygons, etc.
        colors (Sequence[str]): List of hex color strings to use for each sublist.
            Defaults to DEFAULT_COLOR_PALETTE.
        points_annotation_type (int): (POINTS, LINE_LOOP, LINE_LIST, LINE_STRIP).
        thickness (int): PointsAnnotation thickness.

    Returns:
        ImageAnnotations: An ImageAnnotations message containing PointsAnnotations
            for each polygon.
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

    def get_annotation(i: int, point_set) -> PointsAnnotation:
        points = [Point2(x=float(x), y=float(y)) for x, y in point_set]
        outline_color = hex_to_rgba(colors[i % len(colors)])
        r, g, b, a = map(lambda x: x / 255.0, outline_color)
        annotation = PointsAnnotation(
            timestamp=header.stamp,
            type=points_annotation_type,
            points=points,
            outline_color=Color(r=r, g=g, b=b, a=a),
            thickness=thickness,
        )
        return annotation

    point_annotations = []

    for i, point_sets in enumerate(point_sets_list):
        for point_set in point_sets:
            annotation = get_annotation(i, point_set)
            point_annotations.append(annotation)

    image_annotations = ImageAnnotations(points=point_annotations)
    return image_annotations
