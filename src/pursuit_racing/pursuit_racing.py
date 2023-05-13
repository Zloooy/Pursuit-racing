#!/usr/bin/python

from rospy import (
    init_node,
    loginfo,
    spin,
    is_shutdown,
    Publisher,
    Subscriber,
    Rate,
    ServiceProxy,
    wait_for_service,
    get_rostime,
    get_param,
)
from rosnode import rosnode_ping
from typing import Optional, Any, NoReturn, TypeVar, Iterable, SupportsAbs
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from math import sqrt, atan2, pi, copysign

from sys import argv
from dataclasses import dataclass
from itertools import repeat, starmap
from functools import partial
from asyncio import get_event_loop, gather, run, AbstractEventLoop, Future


@dataclass(frozen=True)
class TopicDescription:
    '''Class containing data about topic subscription'''
    topic: str
    listen_type: type


M = TypeVar("M", bound="MultipleMessagesHandler")


class MultipleMessagesHandler:
    '''Handles last n messages from different nodes received during rate tick'''
    rate: Rate
    loop: AbstractEventLoop
    buffer: list[Any]
    timestamps: list[int]
    buffer_ready: Future[None]
    subscribers: list[Subscriber]

    def __init__(self: M, frequency: float, *topics: TopicDescription) -> None:
        self.rate = Rate(frequency)
        self.buffer = list(repeat(None, len(topics)))
        self.timestamps = list(repeat(0, len(topics)))
        self.loop = get_event_loop()
        self.buffer_ready = self.loop.create_future()
        self.subscribers = list(
            starmap(
                lambda i, topic: Subscriber(
                    topic.topic, topic.listen_type, partial(self.receive_callback, i)
                ),
                enumerate(topics),
            )
        )

    def receive_callback(self: M, index: int, value: Any) -> None:
        timestamp = get_rostime().to_nsec()
        # checking if message is least
        if self.timestamps[index] < timestamp:
            # storing last received value
            self.buffer[index] = value
            self.timestamps[index] = timestamp
            if not self.buffer_ready.done() and all(
                map(lambda x: x != None, self.buffer)
            ):
                # unlocking future
                self.loop.call_soon_threadsafe(self.buffer_ready.set_result, None)

    def __aiter__(self: M) -> M:
        return self

    async def __anext__(self: M) -> Iterable[Any]:
        if is_shutdown():
            raise StopAsyncIteration()
        self.rate.sleep()
        await self.buffer_ready
        result = tuple(self.buffer)
        self.buffer_ready = self.loop.create_future()
        return result


C = TypeVar("C", bound="TurtleController")


class TurtleController:
    '''Class controlling one turtle instance spawn and speed'''
    name: str
    initial_position: tuple[float, float, float]
    rate: Rate
    pub: Publisher

    def __init__(
        self: C,
        name: str,
        speed: float,
        initial_position: tuple[int, int, int] = (
            4,
            4,
            0,
        ),
    ) -> None:
        self.name = name
        self.initial_position = initial_position
        self.speed = speed
        self.pub = Publisher(f"/{name}/cmd_vel", Twist, queue_size=10)

    def spawn(self) -> None:
        wait_for_service("/spawn")
        ServiceProxy("/spawn", Spawn)(*self.initial_position, self.name)
        loginfo("spawned")

    def set_velocity(self: C, linear: float = 0, angular: float = 0) -> None:
        vel = Twist()
        vel.linear.x = linear * self.speed
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = angular * self.speed
        self.pub.publish(vel)


A = TypeVar("A", bound="Achilles")


class Achilles:
    '''Class containing follower turtle pursuit logic'''
    followed_name: str
    follower_name: str
    follower_controller: TurtleController
    follower_speed: float
    frequency: float
    follower_angular_speed: SupportsAbs[float]

    def __init__(
        self: A,
        frequency: float = 0.5,
        speed: float = 1,
        followed_name: str = "turtle",
        follower_name: str = "Achilles",
    ) -> None:
        self.followed_name = followed_name
        self.follower_name = follower_name
        self.follower_controller = TurtleController(follower_name, speed)
        self.follower_speed = speed / frequency
        self.follower_angular_speed = speed / (2 * pi) / frequency
        self.frequency = frequency

    async def start(self: A) -> NoReturn:
        wait_for_service("/clear")
        ServiceProxy("/clear", Empty)()
        # Spawning follower
        self.follower_controller.spawn()
        loginfo(f"followed_name {self.followed_name}")
        # Recieving follower and followed poses
        async for (follower_pose, followed_pose) in MultipleMessagesHandler(
            self.frequency,
            TopicDescription(f"/{self.follower_name}/pose", Pose),
            TopicDescription(f"/{self.followed_name}/pose", Pose),
        ):
            distance: float = abs(
                sqrt(
                    (followed_pose.x - follower_pose.x) ** 2
                    + (followed_pose.y - follower_pose.y) ** 2
                )
            )
            if distance > 0.1:
                headingError: float = (
                    atan2(
                        followed_pose.y - follower_pose.y,
                        followed_pose.x - follower_pose.x,
                    )
                    - follower_pose.theta
                )
                if not abs(headingError) < 0.001:
                    # Rotating follower
                    angle: SupportsAbs[float] = 0
                    if headingError > pi:
                        angle = headingError - 2 * pi
                    elif headingError < -pi:
                        angle = headingError + 2 * pi
                    else:
                        angle = headingError
                    angular_speed: float = copysign(
                        min(
                            map(
                                abs,
                                (
                                    self.follower_angular_speed,
                                    angle,
                                ),
                            )
                        ),
                        angle,
                    )
                    self.follower_controller.set_velocity(
                        linear=0, angular=angular_speed
                    )
                else:
                    # Moving follower
                    self.follower_controller.set_velocity(
                        linear=min(self.follower_speed, distance), angular=0
                    )


async def main(args: list[str]) -> NoReturn:
    node_name = "Achilles"
    init_node(node_name)
    a: Achilles = Achilles(
        follower_name=node_name,
        followed_name="turtle",
        frequency=float(get_param("/pursuit_racing/reaction_frequency")),
        speed=float(get_param("/pursuit_racing/speed")),
    )
    await a.start()


if __name__ == "__main__":
    run(main(argv))
