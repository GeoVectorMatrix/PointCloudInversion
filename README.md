# PointCloudInversion

C++ codes for "**S.Xia, D. Chen, J. Peethambaran, P. Wang and S. Xu, Point Cloud Inversion: A Novel Approach for the Localization of Trees in Forests from TLS Data, Remote Sensing.**"

This paper presents a fast and easy-to-use method (Point Cloud Inversion, PCI) for localizing trees in TLS point clouds. The input for PCI is the raw point clouds and no other pre-processing steps (e.g., ground filtering, PCA-based features, noise-removal) are needed. This main contribution of PCI is visually enhancing the pole-like objects in point clouds, and this also making the top-based object localization methods (without model-fitting or PCA-based features) available for TLS data.

Furthermore, PCI can also be treated as a plug-in module in data processing and may benefit other object localization tasks, such as extracting pole-like objects in urban areas.

The experimental results in the TLS-forest benchmark and other challenging scenes (MLS and TLS data of the urban scenes) are shown as follows,

<div align=center><img src="https://github.com/GeoVectorMatrix/PointCloudInversion/blob/main/Images/F1.png" width="500" height="650"/><br/></div>

Please cite this work if the proposed idea helps in your research. 

