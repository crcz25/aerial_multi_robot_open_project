import numpy as np

from typing import List
import itertools
import math


def lse(offsets, measurements, limitx = [-1,8], limity = [-1,8], limitz = [0 , 1.5]):
    '''
        Simple least squares approach for (up to) 4 anchors on the UGv

        Measurements is a list of length num_of_anchors
    '''

    min_err = None
    min_err_pos = [0, 0]

    xys = np.mgrid[limitx[0]:limitx[1]:0.01, limity[0]:limity[1]:0.01, limitz[0]:limitz[1]:0.1].reshape(3, -1).T

    err = None
    for idx, m in enumerate(measurements) :
        if err is None :
            err = ( m - np.linalg.norm(xys - offsets[idx], axis=1) ) ** 2
        else :
            err += ( m - np.linalg.norm(xys - offsets[idx], axis=1) ) ** 2
    # err = ( measurements - numpy.linalg.norm(xys - offsets[idx], axis=1) ) ** 2

    min_err_idx = np.argmin(err)
    min_err = err[min_err_idx]
    min_err_pos = xys[min_err_idx]
    # print(min_err_pos, min_err)
    return min_err_pos, min_err

def mlt_tri_from_measurements_table(
        measurements_table: np.ndarray,
        measurements_cols_names: List[str],
        origin_antenna_1: str,
        origin_antenna_2: str,
        all_antennas: List[str],
) -> List[np.ndarray]:
    """
    - measurements table -> a table that contains the ranges measurements for
    all the possible node pairs in the simulator. Where each row is a moment
    in time and each column is a different range between two nodes.
    - measurements_cols_names -> the names of the columns in the same order
    that are in the table (maybe if it comes in the same file then this is not
    necessary?)
    - origin_antenna_1 -> Name of the antenna that will be used as origin 0, 0
    - origin_antenna_2 -> Name of the antenna that will serve as reference
    to define the origin at origin_antenna_1.
    - all_antennas -> a list with all the antennas names available in the
    simulator (not necessary if we pass all the possible pairs)

    Process:
    1. For each moment we compute the position estimates of all the antennas
    using one antennas as the origin, and the other as a helpful origin
    assistance.
    2. Inside each moment we compute the estimated positions using all the
    antennas as possible bases in the computation.
    """

    # The names of the antennas will correspond to this indices
    # names: [T01, T02, T03]
    # idxs:  [0, 1, 2]
    all_antennas_idxs = [*range(len(all_antennas))]

    all_transformed_positions_all_times = []
    all_sided_positions_all_times = []
    
    for measurements_curr_time in measurements_table:

        positions_for_each_pair_comb = []
        for antenna_as_curr_base_1, antenna_as_curr_base_2 in (
            itertools.combinations(all_antennas, 2)
        ):
            # Get the number of antennas available
            n_antennas = len(all_antennas)

            # Get the index in the antenna list of the pair of current
            # antennas selected as basis.
            curr_base_1_idx = all_antennas.index(antenna_as_curr_base_1)
            curr_base_2_idx = all_antennas.index(antenna_as_curr_base_2)

            # Separate the base antennas from the other antennas and save it
            # in a different list - with the corresponding original index
            remaining_antennas = []
            remaining_antennas_idxs = []
            for idx, antenna in enumerate(all_antennas):
                if (
                    (antenna != antenna_as_curr_base_1) and
                    (antenna != antenna_as_curr_base_2)
                ):
                    remaining_antennas.append(antenna)
                    remaining_antennas_idxs.append(idx)
            
            # Compute the positions for the remaining antennas using the
            # current selected antennas as bases.
            positions_curr_bases = mlt_tri_compute_position_from_two_bases(
                antenna_base_1=antenna_as_curr_base_1,
                ant_base_1_idx=curr_base_1_idx,
                antenna_base_2=antenna_as_curr_base_2,
                ant_base_2_idx=curr_base_2_idx,
                n_antennas=n_antennas,
                remaining_antennas=remaining_antennas,
                remaining_antennas_idxs=remaining_antennas_idxs,
                measurements_curr_time=measurements_curr_time,
                measurements_cols_names=measurements_cols_names
            )

            positions_for_each_pair_comb.append(positions_curr_bases)
        
        # Transform all the positions for each combination pair
        # using the two selected antennas as origin antennas
        trf_positions_for_pair_comb = transform_all_positions(
            all_positions_list=positions_for_each_pair_comb,
            origin_antenna_1=origin_antenna_1,
            origin_antenna_2=origin_antenna_2,
            all_antennas=all_antennas
        )

        all_transformed_positions_all_times.append(trf_positions_for_pair_comb)

        sided_positions_for_pair_comb = manage_side(
            transformed_positions=trf_positions_for_pair_comb
        )
        all_sided_positions_all_times.append(sided_positions_for_pair_comb)

    print('>-------- All sided positions final --------<')
    print(all_sided_positions_all_times)

    return all_sided_positions_all_times


def mlt_tri_compute_position_from_two_bases(
        antenna_base_1: str,
        ant_base_1_idx: str,
        antenna_base_2: str,
        ant_base_2_idx: str,
        n_antennas: int,
        remaining_antennas: List[str],
        remaining_antennas_idxs: List[int],
        measurements_curr_time: np.ndarray,
        measurements_cols_names: List[str],
) -> np.ndarray:
    positions = np.zeros((n_antennas, 5))

    dist_btwn_bases = get_distance_btwn_antennas_from_measurements(
        antenna_1_name=antenna_base_1,
        antenna_2_name=antenna_base_2,
        measurements_cols_names=measurements_cols_names,
        measurements_curr_time=measurements_curr_time
    )

    # Positions of the base antennas
    positions[ant_base_1_idx] = np.array(
        [0, 0, ant_base_1_idx, ant_base_1_idx, ant_base_2_idx]
    )
    positions[ant_base_2_idx] = np.array(
        [dist_btwn_bases, 0, ant_base_2_idx, ant_base_1_idx, ant_base_2_idx]
    )

    for antenna, antenna_idx in zip(
        remaining_antennas, remaining_antennas_idxs
    ):
        dist_base_1_to_antenna = get_distance_btwn_antennas_from_measurements(
            antenna_1_name=antenna_base_1,
            antenna_2_name=antenna,
            measurements_cols_names=measurements_cols_names,
            measurements_curr_time=measurements_curr_time
        )
        dist_base_2_to_antenna = get_distance_btwn_antennas_from_measurements(
            antenna_1_name=antenna_base_2,
            antenna_2_name=antenna,
            measurements_cols_names=measurements_cols_names,
            measurements_curr_time=measurements_curr_time
        )
        cos_theta = (
            dist_btwn_bases ** 2 + dist_base_1_to_antenna ** 2 - dist_base_2_to_antenna ** 2
        ) / (2 * dist_btwn_bases * dist_base_1_to_antenna)

        if cos_theta > 1:
            cos_theta = 1
        elif cos_theta < -1:
            cos_theta = -1
        
        theta = math.acos(cos_theta)
        x_pos = math.cos(theta) * dist_base_1_to_antenna
        y_pos = math.sin(theta) * dist_base_1_to_antenna
        y_m_pos = math.sin(-theta) * dist_base_1_to_antenna

        # Save the antenna position with respect to the bases
        positions[antenna_idx] = np.array(
            [x_pos, y_pos, antenna_idx, ant_base_1_idx, ant_base_2_idx]
        )
    
    return positions

def transform_all_positions(
        all_positions_list: List[np.ndarray],
        origin_antenna_1: str,
        origin_antenna_2: str,
        all_antennas: List[str],
) -> List[np.ndarray]:
    """
    Transform all the points resulting from each combination of base nodes, so
    that all the positions are relative to a selected origin nodes.

    Returns the positions list in the same shape as the original list.
    """
    all_transformed_positions = []

    for positions_from_pair in all_positions_list:
        transformed_positions = np.zeros_like(positions_from_pair)

        # Get the position of the antennas being used as origin for the current
        # positions array - this positions array contains all the estimated
        # positions from each of the antennas being used as a pair (regardless
        # if they are origin or not)
        pos_origin_ant_1_idx = all_antennas.index(origin_antenna_1)
        pos_origin_ant_1 = positions_from_pair[pos_origin_ant_1_idx]
        pos_origin_ant_2_idx = all_antennas.index(origin_antenna_2)
        pos_origin_ant_2 = positions_from_pair[pos_origin_ant_2_idx]

        # Compute the angle between the two origin antennas
        angle_btwn_ant1_ant2 = math.atan2(
            pos_origin_ant_1[1] - pos_origin_ant_2[1],
            pos_origin_ant_1[0] - pos_origin_ant_2[0],
        ) + math.pi

        for ant_pos_idx, ant_position in enumerate(positions_from_pair):
            x_ant_pos = ant_position[0] - pos_origin_ant_1[0]
            y_ant_pos = ant_position[1] - pos_origin_ant_1[1]

            trf_x_ant_pos, trf_y_ant_pos = rotate(
                point=(x_ant_pos, y_ant_pos),
                origin=(0, 0),
                angle=angle_btwn_ant1_ant2
            )

            transformed_positions[ant_pos_idx] = ant_position
            transformed_positions[ant_pos_idx][0] = trf_x_ant_pos
            transformed_positions[ant_pos_idx][1] = trf_y_ant_pos
        
        all_transformed_positions.append(transformed_positions)
    
    return all_transformed_positions

def get_distance_btwn_antennas_from_measurements(
        antenna_1_name: str,
        antenna_2_name: str,
        measurements_cols_names: List[str],
        measurements_curr_time: np.ndarray,
    ):
        """
        Returns the distance between two antennas given their names. The value
        is extracted from the measurements table for the current time (or 
        moment) using a list with the corresponding column names.
        """
        try:
            col_idx_distance_btwn_antennas = measurements_cols_names.index(
                f'from_{antenna_1_name}_to_{antenna_2_name}_range'
            )
            dist_btwn_antennas = measurements_curr_time[
                col_idx_distance_btwn_antennas
            ]
        except ValueError:
            try:
                col_idx_distance_btwn_antennas = measurements_cols_names.index(
                    f'from_{antenna_2_name}_to_{antenna_1_name}_range'
                )
                dist_btwn_antennas = measurements_curr_time[
                    col_idx_distance_btwn_antennas
                ]
            except ValueError:
                raise ValueError(
                    f'Distance between antenna {antenna_1_name} and '
                    f'{antenna_2_name} does not exist in the measurements')
        
        return dist_btwn_antennas

def rotate(point, origin, angle):
    ox, oy = (origin)
    px, py = point
    qx = ox + math.cos(angle) * (px - ox) + math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (ox - px) + math.cos(angle) * (py - oy)
    return qx, qy

def manage_side(transformed_positions):
    """Positive or negative side based on the majority of the points"""
    all_sided_positions = []

    for positions_from_pair in transformed_positions:
        sided_positions = np.zeros_like(positions_from_pair)

        for ant_idx, ant_position in enumerate(positions_from_pair):
            sided_positions[ant_idx] = ant_position

            if ant_position[1] > 0:
                all_sided_positions.append(sided_positions)
            else:
                sided_positions[ant_idx][1] = -1 * ant_position[1]
                all_sided_positions.append(sided_positions)
    
    return all_sided_positions


