<a href="https://www.ultralytics.com/"><img src="https://raw.githubusercontent.com/ultralytics/assets/main/logo/Ultralytics_Logotype_Original.svg" width="320" alt="Ultralytics logo"></a>

# Introduction 🌟

[![Ultralytics Actions](https://github.com/ultralytics/kinect/actions/workflows/format.yml/badge.svg)](https://github.com/ultralytics/kinect/actions/workflows/format.yml)
[![Ultralytics Discord](https://img.shields.io/discord/1089800235347353640?logo=discord&logoColor=white&label=Discord&color=blue)](https://discord.com/invite/ultralytics)
[![Ultralytics Forums](https://img.shields.io/discourse/users?server=https%3A%2F%2Fcommunity.ultralytics.com&logo=discourse&label=Forums&color=blue)](https://community.ultralytics.com/)
[![Ultralytics Reddit](https://img.shields.io/reddit/subreddit-subscribers/ultralytics?style=flat&logo=reddit&logoColor=white&label=Reddit&color=blue)](https://reddit.com/r/ultralytics)

Welcome to the [Ultralytics Kinect](https://github.com/ultralytics/kinect) repository! This project showcases advanced **3D scene reconstruction** algorithms utilizing data captured by the [Microsoft Kinect sensor](https://developer.microsoft.com/en-us/windows/kinect/), a pioneering depth-imaging device. Explore our implementation of cutting-edge techniques in [computer vision](https://www.ultralytics.com/glossary/computer-vision-cv) and see the results in action through example videos on the [Ultralytics YouTube channel](https://www.youtube.com/ultralytics).

[![Kinect Video Preview](https://github.com/ultralytics/kinect/blob/main/preview.jpg)](https://youtu.be/dqK5DkgTGyk "Click to Watch the 3D Reconstruction Demo!")

# Prerequisites 🛠️

Before starting, ensure you have [MATLAB](https://www.mathworks.com/products/matlab.html) version 2018a or newer installed. You'll also need our repository of common MATLAB functions. Clone it using the following command:

```shell
git clone https://github.com/ultralytics/functions-matlab
```

After cloning, add the repository to your MATLAB path. Replace `/path/to/` with the actual directory where you cloned the repository:

```matlab
addpath(genpath('/path/to/functions-matlab'))
```

Additionally, confirm that the following MATLAB toolboxes are installed:

-   Statistics and Machine Learning Toolbox
-   Signal Processing Toolbox

For more details on setting up development environments, check out the [Ultralytics documentation](https://docs.ultralytics.com/).

# How to Run 🏃

To launch the 3D scene reconstruction process, simply run the following command within your MATLAB environment:

```matlab
buildscene
```

This script initiates the reconstruction using the provided Kinect data and algorithms. For a general overview of running Ultralytics projects, see our [Quickstart Guide](https://docs.ultralytics.com/quickstart/).

# Contribute 🤝

Your contributions can significantly enhance this project! We welcome collaboration from the open-source community. To get started, please review our [Contributing Guide](https://docs.ultralytics.com/help/contributing/) and consider sharing your feedback through our [Survey](https://www.ultralytics.com/survey?utm_source=github&utm_medium=social&utm_campaign=Survey). Your insights help drive improvements and innovation at [Ultralytics](https://www.ultralytics.com/). A big thank you to all our contributors!

[![Ultralytics open-source contributors](https://raw.githubusercontent.com/ultralytics/assets/main/im/image-contributors.png)](https://github.com/ultralytics/ultralytics/graphs/contributors)

# Licensing Options ⚖️

This software is available under the **AGPL-3.0 License**, an [OSI-approved](https://opensource.org/license/agpl-v3) open-source license emphasizing transparency and collaboration. This license is ideal for students, researchers, and enthusiasts. You can find the full license details in the [LICENSE](LICENSE) file.

For commercial use cases, Ultralytics offers an **Enterprise License**. This allows seamless integration of our software into your commercial products and services, bypassing the open-source requirements of AGPL-3.0. Learn more about commercial licensing options at [Ultralytics Licensing](https://www.ultralytics.com/license). Understanding different [open-source licenses](https://opensource.org/licenses) can be helpful.

# Stay Connected! 🌐

Encountered a bug or have a feature request? Please submit it via [GitHub Issues](https://github.com/ultralytics/kinect/issues). We also invite you to join our vibrant [Discord](https://discord.com/invite/ultralytics) community for discussions, support, and collaboration with fellow developers and the Ultralytics team. Stay updated on the latest in AI and computer vision through the [Ultralytics Blog](https://www.ultralytics.com/blog).

<br>
<div align="center">
  <a href="https://github.com/ultralytics"><img src="https://github.com/ultralytics/assets/raw/main/social/logo-social-github.png" width="3%" alt="Ultralytics GitHub"></a>
  <img src="https://github.com/ultralytics/assets/raw/main/social/logo-transparent.png" width="3%" alt="space">
  <a href="https://www.linkedin.com/company/ultralytics/"><img src="https://github.com/ultralytics/assets/raw/main/social/logo-social-linkedin.png" width="3%" alt="Ultralytics LinkedIn"></a>
  <img src="https://github.com/ultralytics/assets/raw/main/social/logo-transparent.png" width="3%" alt="space">
  <a href="https://twitter.com/ultralytics"><img src="https://github.com/ultralytics/assets/raw/main/social/logo-social-twitter.png" width="3%" alt="Ultralytics Twitter"></a>
  <img src="https://github.com/ultralytics/assets/raw/main/social/logo-transparent.png" width="3%" alt="space">
  <a href="https://youtube.com/ultralytics"><img src="https://github.com/ultralytics/assets/raw/main/social/logo-social-youtube.png" width="3%" alt="Ultralytics YouTube"></a>
  <img src="https://github.com/ultralytics/assets/raw/main/social/logo-transparent.png" width="3%" alt="space">
  <a href="https://www.tiktok.com/@ultralytics"><img src="https://github.com/ultralytics/assets/raw/main/social/logo-social-tiktok.png" width="3%" alt="Ultralytics TikTok"></a>
  <img src="https://github.com/ultralytics/assets/raw/main/social/logo-transparent.png" width="3%" alt="space">
  <a href="https://ultralytics.com/bilibili"><img src="https://github.com/ultralytics/assets/raw/main/social/logo-social-bilibili.png" width="3%" alt="Ultralytics BiliBili"></a>
  <img src="https://github.com/ultralytics/assets/raw/main/social/logo-transparent.png" width="3%" alt="space">
  <a href="https://discord.com/invite/ultralytics"><img src="https://github.com/ultralytics/assets/raw/main/social/logo-social-discord.png" width="3%" alt="Ultralytics Discord"></a>
</div>
