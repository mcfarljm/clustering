# Copyright 2020 D-Wave Systems Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import math

import dwave.inspector
from dwave.system import LeapHybridDQMSampler
from dimod import DiscreteQuadraticModel

from utilities import get_groupings, visualize_groupings, visualize_scatterplot


# Note: use of a class to store the coordinates is a remnant of the
# BQM formulation
class Coordinate:
    def __init__(self, x, y):
        self.x = x
        self.y = y

def get_distance(coordinate_0, coordinate_1):
    diff_x = coordinate_0.x - coordinate_1.x
    diff_y = coordinate_0.y - coordinate_1.y

    return math.sqrt(diff_x**2 + diff_y**2)


def get_max_distance(coordinates):
    max_distance = 0
    for i, coord0 in enumerate(coordinates[:-1]):
        for coord1 in coordinates[i+1:]:
            distance = get_distance(coord0, coord1)
            max_distance = max(max_distance, distance)

    return max_distance


def cluster_points(scattered_points, filename, problem_inspector):
    """Perform clustering analysis on given points

    Args:
        scattered_points (list of tuples):
            Points to be clustered
        filename (str):
            Output file for graphic
        problem_inspector (bool):
            Whether to show problem inspector
    """
    # Set up problem
    # Note: max_distance gets used in division later on. Hence, the max(.., 1)
    #   is used to prevent a division by zero
    coordinates = [Coordinate(x, y) for x, y in scattered_points]
    max_distance = max(get_max_distance(coordinates), 1)


    # Initialize discrete quadratic model
    dqm = DiscreteQuadraticModel()
    for coord in coordinates:
        dqm.add_variable(3)

    # Edit DQM to bias for close together points to share the same color
    for i, coord0 in enumerate(coordinates[:-1]):
        for j in range(i+1, len(coordinates)):
            coord1 = coordinates[j]
            # Set up weight
            d = get_distance(coord0, coord1) / max_distance  # rescale distance
            weight = -math.cos(d*math.pi)

            # Apply weights to BQM
            for icolor in range(3):
                dqm.set_quadratic_case(i, icolor, j, icolor, weight)

    # Edit BQM to bias for far away points to have different colors
    for i, coord0 in enumerate(coordinates[:-1]):
        for j in range(i+1, len(coordinates)):
            coord1 = coordinates[j]
            # Set up weight
            # Note: rescaled and applied square root so that far off distances
            #   are all weighted approximately the same
            d = math.sqrt(get_distance(coord0, coord1) / max_distance)
            weight = -math.tanh(d) * 0.1

            # Apply weights to BQM
            dqm.set_quadratic_case(i, 0, j, 1, weight)
            dqm.set_quadratic_case(i, 0, j, 2, weight)
            dqm.set_quadratic_case(i, 1, j, 0, weight)
            dqm.set_quadratic_case(i, 1, j, 2, weight)
            dqm.set_quadratic_case(i, 2, j, 0, weight)
            dqm.set_quadratic_case(i, 2, j, 1, weight)

    # Submit problem to D-Wave sampler
    sampler = LeapHybridDQMSampler()
    sampleset = sampler.sample_dqm(dqm)

    best_sample = sampleset.first.sample

    # Visualize solution
    groupings = get_groupings(best_sample, coordinates)
    visualize_groupings(groupings, filename)

    # Print solution onto terminal
    # Note: This is simply a more compact version of 'best_sample'
    print(groupings)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('--no-problem-inspector', action='store_false', dest='problem_inspector', help='do not show problem inspector')
    args = parser.parse_args()
    
    # Simple, hardcoded data set
    scattered_points = [(0, 0), (1, 1), (2, 4), (3, 2)]

    # Save the original, un-clustered plot
    orig_filename = "four_points.png"
    visualize_scatterplot(scattered_points, orig_filename)

    # Find clusters
    # Note: the key part of this demo is in the construction of this function
    clustered_filename = "four_points_clustered.png"
    cluster_points(scattered_points, clustered_filename, args.problem_inspector)

    print("Your plots are saved to '{}' and '{}'.".format(orig_filename,
                                                     clustered_filename))
