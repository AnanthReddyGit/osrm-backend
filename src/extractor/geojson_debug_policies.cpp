#include "extractor/geojson_debug_policies.hpp"
#include "util/coordinate.hpp"
#include "util/geojson_debug_policy_toolkit.hpp"

#include <algorithm>

namespace osrm
{
namespace extractor
{

IntersectionShapePrinter::IntersectionShapePrinter(
    const util::NodeBasedDynamicGraph &node_based_graph,
    const std::vector<extractor::QueryNode> &node_coordinates,
    const extractor::guidance::CoordinateExtractor &coordinate_extractor)
    : node_based_graph(node_based_graph), node_coordinates(node_coordinates),
      coordinate_extractor(coordinate_extractor)
{
}

util::json::Array IntersectionShapePrinter::
operator()(const NodeID intersection_node,
           const extractor::guidance::IntersectionShape &intersection,
           const boost::optional<util::json::Object> &node_style,
           const boost::optional<util::json::Object> &way_style) const
{
    // request the number of lanes. This process needs to be in sync with what happens over at
    // intersection_generator
    const auto intersection_lanes =
        intersection.FindMaximum(guidance::makeExtractLanesForRoad(node_based_graph));

    std::vector<util::Coordinate> coordinates;
    coordinates.reserve(intersection.size());
    coordinates.push_back(node_coordinates[intersection_node]);

    const auto road_to_coordinate =
        [&](const auto &road) {
            const constexpr auto FORWARD = false;
            const auto to_node = node_based_graph.GetTarget(road.eid);
            return coordinate_extractor.GetCoordinateAlongRoad(
                intersection_node, road.eid, FORWARD, to_node, intersection_lanes);
        };

    std::transform(intersection.begin(),
                   intersection.end(),
                   std::back_inserter(coordinates),
                   road_to_coordinate);

    util::json::Array features;
    features.values.push_back(
        util::makeFeature("MultiPoint", makeJsonArray(coordinates), node_style));

    if (coordinates.size() > 1)
    {
        std::vector<util::Coordinate> line_coordinates(2);
        line_coordinates[0] = coordinates.front();
        const auto coordinate_to_line = [&](const util::Coordinate coordinate) {
            line_coordinates[1] = coordinate;
            return util::makeFeature("LineString", makeJsonArray(line_coordinates), way_style);
        };

        std::transform(std::next(coordinates.begin()),
                       coordinates.end(),
                       std::back_inserter(features.values),
                       coordinate_to_line);
    }
    return features;
}

IntersectionPrinter::IntersectionPrinter(
    const util::NodeBasedDynamicGraph &node_based_graph,
    const std::vector<extractor::QueryNode> &node_coordinates,
    const extractor::guidance::CoordinateExtractor &coordinate_extractor)
    : IntersectionShapePrinter(node_based_graph, node_coordinates, coordinate_extractor)
{
}

util::json::Array IntersectionPrinter::
operator()(const NodeID intersection_node,
           const extractor::guidance::Intersection &intersection,
           const boost::optional<util::json::Object> &node_style,
           const boost::optional<util::json::Object> &way_style) const
{
    guidance::IntersectionShape intersection_shape;
    intersection_shape.resize(intersection.size());
    std::transform(intersection.begin(),
                   intersection.end(),
                   intersection_shape.begin(),
                   [](const auto &road) { return guidance::IntersectionShapeData(road); });
    return Base::operator()(intersection_node, intersection_shape, node_style, way_style);
}

} /* namespace extractor */
} /* namespace osrm */
