from tools.page4_cloud_reference import (
    CloudPoint,
    RenderMode,
    choose_mode,
    project_point,
    should_hold_previous_scene,
    summarize_cloud,
)


def test_choose_mode_single_target_when_points_share_cluster():
    points = [
        CloudPoint(cluster_index=1, x=0.10, y=1.20, z=0.90, speed=0.0),
        CloudPoint(cluster_index=1, x=0.14, y=1.24, z=1.02, speed=0.0),
        CloudPoint(cluster_index=1, x=0.08, y=1.16, z=0.84, speed=0.0),
    ]

    assert choose_mode(points) is RenderMode.SINGLE


def test_choose_mode_multi_target_when_multiple_clusters_exist():
    points = [
        CloudPoint(cluster_index=1, x=0.10, y=1.20, z=0.90, speed=0.0),
        CloudPoint(cluster_index=2, x=-0.40, y=2.10, z=0.65, speed=0.0),
    ]

    assert choose_mode(points) is RenderMode.MULTI


def test_project_point_uses_fixed_oblique_mapping():
    point = CloudPoint(cluster_index=1, x=0.50, y=1.00, z=1.20, speed=0.0)
    screen_x, screen_y = project_point(
        point,
        center_x=64,
        floor_y=55,
        sx=18,
        sy=6,
        sz=16,
        dy=4,
    )

    assert (screen_x, screen_y) == (67, 31)


def test_summarize_cloud_reports_distance_height_and_count():
    points = [
        CloudPoint(cluster_index=1, x=0.10, y=1.80, z=1.10, speed=0.0),
        CloudPoint(cluster_index=1, x=0.16, y=1.76, z=0.92, speed=0.0),
        CloudPoint(cluster_index=1, x=0.07, y=1.85, z=1.01, speed=0.0),
    ]

    summary = summarize_cloud(points)

    assert summary["point_count"] == 3
    assert summary["max_height"] == 1.10
    assert round(summary["nearest_distance"], 2) == 1.76


def test_single_target_summary_uses_one_cluster():
    points = [
        CloudPoint(cluster_index=7, x=0.10, y=1.80, z=1.10, speed=0.0),
        CloudPoint(cluster_index=7, x=0.18, y=1.72, z=0.98, speed=0.0),
        CloudPoint(cluster_index=7, x=0.06, y=1.84, z=0.91, speed=0.0),
    ]

    summary = summarize_cloud(points)

    assert summary["cluster_count"] == 1
    assert round(summary["centroid_x"], 2) == 0.11
    assert round(summary["centroid_y"], 2) == 1.79
    assert round(summary["centroid_z"], 2) == 1.00


def test_single_target_downsamples_points_without_losing_extrema():
    points = [
        CloudPoint(cluster_index=1, x=-0.20, y=1.30, z=0.70, speed=0.0),
        CloudPoint(cluster_index=1, x=-0.10, y=1.35, z=0.90, speed=0.0),
        CloudPoint(cluster_index=1, x=0.00, y=1.40, z=1.10, speed=0.0),
        CloudPoint(cluster_index=1, x=0.10, y=1.45, z=1.00, speed=0.0),
        CloudPoint(cluster_index=1, x=0.20, y=1.50, z=0.80, speed=0.0),
    ]

    sample = summarize_cloud(points)["sample_points"]

    assert len(sample) <= 4
    assert sample[0].x == -0.20
    assert sample[-1].x == 0.20


def test_multi_target_mode_prefers_cluster_aggregation():
    points = [
        CloudPoint(cluster_index=1, x=0.10, y=1.20, z=0.90, speed=0.0),
        CloudPoint(cluster_index=1, x=0.12, y=1.25, z=0.95, speed=0.0),
        CloudPoint(cluster_index=2, x=-0.45, y=2.00, z=0.60, speed=0.0),
    ]

    summary = summarize_cloud(points)

    assert summary["cluster_count"] == 2
    assert summary["mode"] is RenderMode.MULTI


def test_empty_cloud_does_not_clear_recent_scene_immediately():
    assert should_hold_previous_scene(last_nonzero_ms=1000, now_ms=2200, hold_ms=1500) is True
    assert should_hold_previous_scene(last_nonzero_ms=1000, now_ms=2600, hold_ms=1500) is False
