////////////////////////////////////////////////////////////////////////////////
/// @file           UnionFind.h
/// @brief          A program to define a UnionFind tree
/// @author         Keisuke KIMURA
/// @date           2020/06/01
/// $Version:       1.0.0
/// @note           https://en.wikipedia.org/wiki/Disjoint-set_data_structure
/// @par            History
///                 2020/06/01 : Support for doxygen
///
/// Copyright (c) 2020 Keisuke KIMURA. All Rights reserved.
/// This software is released under the MIT License.
/// http://opensource.org/licenses/mit-license.php
///
////////////////////////////////////////////////////////////////////////////////

#ifndef TELLO_CONTROLLER_UNIONFIND_H
#define TELLO_CONTROLLER_UNIONFIND_H

/**
 * @namespace tello_controller
 * @brief ROS self-made package "tello_controller".
 */
namespace tello_controller {
    /**
     * @struct UnionFind
     * @brief
     *  It provides near-constant-time operations (bounded by the inverse Ackermann function) to
     *  add new sets, to merge existing sets, and to determine whether elements are in the same set.
     */
    struct UnionFind {
        std::vector<int> data;

        UnionFind() = default;

        UnionFind(int size) { init(size); }

        /**
         * @brief initializer
         * @param size Maximum number of elements
         */
        void init(int size) {
            data.clear();
            data.assign(size, -1);
        }

        /**
         * @brief Unite two sets into one
         * @param x vertex 1
         * @param y vertex 2
         * @return Return 'false' if x and y are the same set, and 'true' otherwise.
         */
        bool unite(int x, int y) {
            x = root(x);
            y = root(y);
            if (x != y) {
                if (data[y] < data[x]) std::swap(x, y);
                data[x] += data[y];
                data[y] = x;
            }
            return x != y;
        }

        /**
         * @brief Check to see if x and y are the same set.
         * @param x vertex 1
         * @param y vertex 2
         * @return Return 'true' if x and y are the same set, and 'false' otherwise.
         */
        bool same(int x, int y) {
            return root(x) == root(y);
        }

        /**
         * @brief Find the representative point of a set
         * @param x vertex
         * @return　representative point
         */
        int root(int x) {
            return data[x] < 0 ? x : data[x] = root(data[x]);
        }
        /**
         * @brief check the size of a set
         * @param x vertex
         * @return The size of the set containing x
         */
        int size(int x) {
            return -data[root(x)];
        }
    };
}

#endif //TELLO_CONTROLLER_UNIONFIND_H
