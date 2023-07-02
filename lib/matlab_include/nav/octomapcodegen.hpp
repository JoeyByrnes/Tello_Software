/* Copyright 2019-2022 The MathWorks, Inc. */

#ifndef OCTOMAPCODEGEN_H_
#define OCTOMAPCODEGEN_H_

#ifdef BUILDING_LIBMWOCTOMAPCODEGEN
#include "octomapcodegen/octomapcodegen_util.hpp"
#else
#include "octomapcodegen_util.hpp"
#endif

#include <memory>
#include <fstream>
#include <string>

// Forward declaration
namespace octomap {
    class OcTree;
}

namespace nav {
    
    class OCTOMAP_CODEGEN_API octomapcodegen{
        
    public:
        
        octomapcodegen(real64_T resolution);
        ~octomapcodegen();
        
        real64_T octomapGetResolution();
        
        void octomapSetResolution(real64_T resolution);

        real64_T octomapGetOccupiedThreshold();

        void octomapSetOccupiedThreshold(real64_T probability);

        real64_T octomapGetFreeThreshold();

        void octomapSetFreeThreshold(real64_T probability);

        uint32_T octomapGetNumLeafNodes();
        
        void octomapGetMapDimensions(real64_T* dimensions);
        
        void octomapUpdateNodeBoolean(real64_T* xyz, boolean_T occupied, boolean_T lazyEval);
        
        void octomapUpdateNodeDouble(real64_T* xyz, real64_T probUpdate, boolean_T lazyEval);
        
        void octomapSetNodeValue(real64_T* xyz, real64_T prob, boolean_T lazyEval);
        
        void octomapGetOccupancy(real64_T* xyz, uint32_T nRows, real64_T* occupancyValues);

        void octomapCheckOccupancy(real64_T* xyz, uint32_T nRows, real64_T* ternaryValues);
        
        void octomapUpdateInternalOccupancy();
        
        void octomapPrune();
        
        void octomapToMaxLikelihood();
             
        void octomapExtractVisualizationData(real64_T maxDepth, real64_T* outData);
        
        uint32_T octomapMmemoryUsage();
        
        void octomapInflate(real64_T radius, real64_T occThreshold);
        
        void octomapSetClampingThreshold(real64_T clampingThresMin, real64_T clampingThresMax);
        
        boolean_T octomapReadBinary(const char* filename);
        
        void octomapWriteBinary(const char* filename);
        
        boolean_T octomapRead(const char* filename);
        
        void octomapWrite(const char* filename);
        
        real64_T octomapGetProbHit();
        
        real64_T octomapGetProbMiss();
        
        uint32_T octomapGetSizeSerializationData();

        void octomapSerialization(char* data);
        
        boolean_T octomapDeserialization(char* data, uint32_T n);
        
        void octomapDeserializationFullROSMsgData(real64_T resolution, char* data, uint32_T n);
        
        void octomapDeserializationBinaryROSMsgData(real64_T resolution, char* data, uint32_T n);
        
        void octomapGetRayIntersection(
            real64_T* ptStart,
            real64_T* ptDirections,
            uint32_T nPtDirection,
            real64_T occupiedThreshold,
            boolean_T ignoreUnknownCells,
            real64_T maxRange,
            real64_T* outData);
        
        bool insertPointCloudImpl_real64(real64_T* origin,
                real64_T* points,
                uint32_T nPoints,
                real64_T maxRange,
                boolean_T lazyEval,
                boolean_T discretize);

        bool insertPointCloudImpl_real32(real64_T* origin,
                real32_T* points,
                uint32_T nPoints,
                real64_T maxRange,
                boolean_T lazyEval,
                boolean_T discretize);

        std::shared_ptr<octomap::OcTree> getTree()
        {
            return _map;
        }

    private:
        std::shared_ptr<octomap::OcTree> _map; ///< Octomap Map Object

        double occ_free_thres; /// free threshold (probability)

        /// Used to read from Char array
        class OneShotReadBuf : public std::streambuf {
        public:
            OneShotReadBuf(char* s, uint32_T n) {
                size_t n1 = static_cast<size_t>(n);
                setg(s, s, s + n1);
            }
        };
        
        // read from istream (binary)
        bool readBinaryImpl(std::istream& buf);

        // write to ostream (binary)
        void writeBinaryImpl(std::ostream& buf);

        // write to ostream
        void writeImpl(std::ostream& buf);

        // read from istream
        bool readImpl(std::istream& buf);
        
        template <typename TfloatPoint>
                bool insertPointCloudImpl(double const* origin,
                TfloatPoint const* points,
                uint32_T rows,
                double maxRange,
                bool lazyEval,
                bool discretize);
    };
} // namespace nav

#endif /* OCTOMAPCODEGEN_H_ */
