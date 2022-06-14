export interface Configuration {
    ros1: {
        masterUri: string,
    },
    ros2: {
        domainId: number,
    },
    default: Middleware,
}

export enum Middleware {
    ROS1 = 'ROS1',
    ROS2 = 'ROS2'
}