from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
import carla


def visualise_route(route: list[carla.Location], lifetime=600):
    """
    Visualises the given route using a simple text-based representation.

    Args:
        route ([carla.Location]): A list of coordinates (carla.Location) representing the route.
        lifetime (int): Lifetime of waypoints
    """

    world_debug = CarlaDataProvider.get_world().debug

    line_color = carla.Color(r=255, g=0, b=0)
    waypoint_color = carla.Color(r=0, g=0, b=255)

    for waypoint in range(len(route) - 1):
        begin = route[waypoint]
        end = route[waypoint + 1]

        world_debug.draw_line(
            begin=begin, end=end, thickness=0.5, color=line_color, life_time=lifetime
        )
        world_debug.draw_point(
            location=begin, size=0.1, color=waypoint_color, life_time=lifetime
        )

    # draw final waypoint
    world_debug.draw_point(
        location=route[-1], size=0.1, color=waypoint, life_time=lifetime
    )
