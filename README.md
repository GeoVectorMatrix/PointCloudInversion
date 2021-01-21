# PointCloudInversion

C++ codes for "**Xia, S.; Chen, D.; Peethambaran, J.; Wang, P.; Xu, S\*. Point Cloud Inversion: A Novel Approach for the Localization of Trees in Forests from TLS Data. Remote Sens. 2021, 13, 338. https://doi.org/10.3390/rs13030338.**"

Point Cloud Inversion (PCI) is a fast and easy-to-use method for localizing trees in TLS point clouds. The input for PCI is the raw point clouds and no other pre-processing steps (e.g., ground filtering, PCA-based features, noise-removal) are needed. This main contribution of PCI is visually enhancing the pole-like objects in point clouds, and this also making the top-based object localization methods (without model-fitting or PCA-based features) available for TLS data.

Furthermore, PCI can also be treated as a plug-in module in data processing and may benefit other object localization tasks, such as extracting pole-like objects in urban areas.

The experimental results in the TLS-forest benchmark and other challenging scenes (MLS and TLS data of the urban scenes) are shown as follows,

<div align=center><img src="https://github.com/GeoVectorMatrix/PointCloudInversion/blob/main/Images/F1.png" width="500" height="650"/><br/></div>

Please cite this work if the proposed idea helps in your research. 
```
@Article{rs13030338,
AUTHOR = {Xia, Shaobo and Chen, Dong and Peethambaran, Jiju and Wang, Pu and Xu, Sheng},
TITLE = {Point Cloud Inversion: A Novel Approach for the Localization of Trees in Forests from TLS Data},
JOURNAL = {Remote Sensing},
VOLUME = {13},
YEAR = {2021},
NUMBER = {3},
ARTICLE-NUMBER = {338},
DOI = {10.3390/rs13030338}
}
```
