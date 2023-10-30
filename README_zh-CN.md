# Lily

Lily is a collection of tools for the calibration of lidar sensors.

You might be wondering why the project is named as it is, given its function seems unrelated. At its core, this is a lidar2lidar calibration tool. While 'Lily' typically connotes a person's name, its origin here is a bit more nuanced. When I was conceptualizing this project, it struck me that 'lidar2lidar' phonetically echoes 'double li'. This inspired the name 'Lily'. Additionally, 'Lily' is also the title of a song I deeply admire, crafted by the Chinese artist, Li Jian. If it piques your curiosity, I'd recommend exploring his work.

## Requirements

- ros1

## Usage

本标定工具分自动模式与手动模式

### Auto Mode
>
> Note : 自动标定分为多个步骤，只有标定成功的步骤才会更新标定参数，如果标定失败，则不会更新参数且会在当前步骤结束后退出标定

在自动模式下，标定工具会自动完成以下步骤：

1. 对每个lidar进行地面标定，默认设置地面高度为0

2. 对每个lidar向着主lidar进行标定

#### 手动选点
>
> 在 lidar 进行自动标定时候，如果lidar打在地面的点比较稀疏，或者因为遮挡等原因，导致地面点不够，会导地面检测失败，从而导致致标定失败，此时可以通过手动选点的方式来标定，我们通过rviz来辅助取点，具体步骤如下：

首先，打印 `/clicked_point`信息

```bash
rostopic echo /clicked_point
```

### Mannual Mode
