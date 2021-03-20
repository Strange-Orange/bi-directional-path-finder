#ifndef _INDEXED_PRIORITY_QUEUE_H
#define _INDEXED_PRIORITY_QUEUE_H

#include <functional>
#include <stdexcept>
#include <vector>
#include <unordered_map>

template <typename T, typename Hashfunc=std::hash<T>>
class IndexedPriorityQueue
{
    private:
        std::vector<T> m_heap;
        std::unordered_map<T, size_t, Hashfunc> m_heapMap;

        // Swap two items in the heap and heap map
        void swap(size_t p_index0, size_t p_index1)
        {
            T l_heapTemp = m_heap.at(p_index0);
            size_t l_heapMapTemp = m_heapMap.at(m_heap.at(p_index0));

            m_heapMap.at(m_heap.at(p_index0)) = m_heapMap.at(m_heap.at(p_index1));
            m_heap.at(p_index0) = m_heap.at(p_index1);
            m_heapMap.at(m_heap.at(p_index1)) = l_heapMapTemp;
            m_heap.at(p_index1) = l_heapTemp;
        }

    public:
        IndexedPriorityQueue() {}

        IndexedPriorityQueue(const std::vector<T>& p_arr)
            : m_heap(p_arr)
        {
            // Make sure there are no duplicates in the vector
            std::unordered_map<T, int> l_values;
            size_t uniqueValues = 0;
            for (size_t i = 0; i < m_heap.size(); i++)
            {
                if (l_values.find(m_heap.at(i)) == l_values.end())
                {
                    l_values[m_heap.at(i)] = i;
                    m_heap.at(uniqueValues) = m_heap.at(i);
                    m_heapMap[m_heap.at(i)] = uniqueValues++;
                }
            }
            m_heap.erase(m_heap.begin() + uniqueValues, m_heap.end());

            if (!m_heap.empty())
            {
                build_min_heap();
            }
        }
        ~IndexedPriorityQueue() {}

        void build_min_heap()
        {
            for (int i = m_heap.size() / 2; i >= 0; i--)
            {
                min_heapify(i);
            }
        }

        size_t size() const 
        {
            return m_heap.size();
        }

        bool empty() const
        {
            return m_heap.size() == 0;
        }

        void min_heapify(size_t p_index)
        {
            bubble_down(p_index);
        }

        T peek() const
        {
            if (!m_heap.empty())
            {
                return m_heap.at(0);
            }
            else
            { 
                throw std::out_of_range("Priority queue is empty");
            }
        }

        T extract_min()
        {
            T l_min = m_heap.at(0);
            swap(0, m_heap.size() - 1);
            m_heap.erase(m_heap.begin() + (m_heap.size() - 1));
            m_heapMap.erase(l_min);
            bubble_down(0);
            return l_min;
        }

        void insert(T p_item)
        {
            // Don't insert duplicates
            if (m_heapMap.find(p_item) == m_heapMap.end())
            {
                m_heap.push_back(p_item);
                m_heapMap[p_item] = m_heap.size() - 1;
                bubble_up(m_heap.size() - 1);
            }
        }

        void bubble_up(int p_index)
        {
            int l_parent = (p_index - 1) / 2;
            if (l_parent >= 0 && m_heap.at(l_parent) > m_heap.at(p_index))
            {
                swap(p_index, l_parent);
                bubble_up(l_parent);
            }
        }

        void bubble_down(int p_index)
        {
            size_t l_leftChild = p_index * 2 + 1;
            size_t l_rightChild = p_index * 2 + 2;
            int l_smallest = p_index;
            if (l_leftChild < m_heap.size() && m_heap.at(l_leftChild) < m_heap.at(l_smallest))
                l_smallest = l_leftChild;
            if (l_rightChild < m_heap.size() && m_heap.at(l_rightChild) < m_heap.at(l_smallest))
                l_smallest = l_rightChild;

            if (l_smallest != p_index)
            {
                swap(p_index, l_smallest);
                bubble_down(l_smallest);
            }
        }

        // At the moment this just replaces the item with a new one. There has to be a better solution for large objects
        void change_priority(T p_old, T p_new)
        {
            if (m_heapMap.find(p_old) != m_heapMap.end())
            {
                int l_oldPos = m_heapMap.at(p_old);
                m_heap.at(l_oldPos) = p_new;
                m_heapMap.erase(p_old);
                m_heapMap[p_new] = l_oldPos;

                bubble_up(l_oldPos);
                bubble_down(l_oldPos);
            }
        }
};

#endif