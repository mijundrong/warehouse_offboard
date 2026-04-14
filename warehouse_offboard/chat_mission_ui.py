#!/usr/bin/env python3

import threading
import time

import pygame
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from warehouse_offboard.llm_selector import TargetSelector


class MissionUiBridge(Node):
    def __init__(self):
        super().__init__('mission_ui_bridge')

        self.target_pub = self.create_publisher(
            String,
            '/mission_target_name',
            10
        )

        self.status_sub = self.create_subscription(
            String,
            '/mission_status_text',
            self.status_callback,
            10
        )

        self.selector = TargetSelector(['A-01', 'A-02', 'A-03'])

        self.latest_status = 'WAITING_FOR_COMMAND'
        self.status_history = ['시스템 준비 완료. 목적지를 입력하세요.']

        self.get_logger().info('Mission UI Bridge initialized')

    def status_callback(self, msg: String):
        self.latest_status = msg.data

        if msg.data.startswith('MISSION_STARTED:'):
            target = msg.data.split(':', 1)[1]
            self.status_history.insert(0, f'미션 시작: {target}')
        elif msg.data.startswith('MISSION_FINISHED:'):
            target = msg.data.split(':', 1)[1]
            self.status_history.insert(0, f'미션 완료: {target}')
        elif msg.data.startswith('MISSION_REJECTED:'):
            reason = msg.data.split(':', 1)[1]
            self.status_history.insert(0, f'미션 거부: {reason}')
        else:
            self.status_history.insert(0, msg.data)

        self.status_history = self.status_history[:10]

    def publish_target(self, target_name: str):
        msg = String()
        msg.data = target_name
        self.target_pub.publish(msg)
        self.get_logger().info(f'Published mission target: {target_name}')


class LLMInterface:
    def __init__(self, ros_node: MissionUiBridge):
        self.ros_node = ros_node

        pygame.init()
        pygame.key.start_text_input() #한글 입력을 위해 추가됨

        self.width, self.height = 800, 600
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("Drone Mission Chat UI")

        self.font_large = pygame.font.SysFont('Noto Sans CJK KR', 36, bold=True)
        self.font_medium = pygame.font.SysFont('Noto Sans CJK KR', 28)
        self.font_small = pygame.font.SysFont('Noto Sans CJK KR', 20)

        self.colors = {
            'background': (30, 30, 40),
            'title': (200, 200, 255),
            'text': (255, 255, 255),
            'highlight': (255, 255, 0),
            'info': (100, 255, 100),
            'warning': (255, 100, 100),
            'input_bg': (50, 50, 60),
            'input_text': (255, 255, 255),
            'command_bg': (40, 60, 40),
            'response_bg': (60, 40, 70),
            'status_bg': (45, 45, 65),
        }

        self.input_text = ""
        self.input_active = True
        self.input_rect = pygame.Rect(50, 500, 700, 40)

        self.command_history = []
        self.responses = []
        self.max_history = 5

        self.max_text_width = self.width - 150

        print("Drone mission chat UI initialized.")
        print("Example commands: 'A-01', '1', '2', 'A03'")

    def truncate_text(self, text, font, max_width):
        if font.size(text)[0] <= max_width:
            return text

        ellipsis = "..."
        ellipsis_width = font.size(ellipsis)[0]
        available_width = max_width - ellipsis_width

        left, right = 0, len(text)
        while left < right:
            mid = (left + right) // 2
            if font.size(text[:mid])[0] <= available_width:
                left = mid + 1
            else:
                right = mid

        return text[:left - 1] + ellipsis

    def render_ui(self):
        self.screen.fill(self.colors['background'])

        title = self.font_large.render("Drone Natural Language Control", True, self.colors['title'])
        self.screen.blit(title, (self.width // 2 - title.get_width() // 2, 20))

        y_pos = 80

        header = self.font_medium.render("Mission Status:", True, self.colors['info'])
        self.screen.blit(header, (50, y_pos))
        y_pos += 40

        status_bg = pygame.Rect(60, y_pos, self.width - 120, 35)
        pygame.draw.rect(self.screen, self.colors['status_bg'], status_bg)

        latest_status = self.truncate_text(
            self.ros_node.latest_status,
            self.font_small,
            self.width - 160
        )
        status_text = self.font_small.render(latest_status, True, self.colors['text'])
        self.screen.blit(status_text, (70, y_pos + 7))
        y_pos += 55

        sub_header = self.font_small.render("Recent System Messages:", True, self.colors['highlight'])
        self.screen.blit(sub_header, (50, y_pos))
        y_pos += 28

        history_to_show = self.ros_node.status_history[:3]
        if history_to_show:
            for msg in history_to_show:
                line = self.truncate_text(msg, self.font_small, self.width - 140)
                line_text = self.font_small.render(line, True, self.colors['text'])
                self.screen.blit(line_text, (70, y_pos))
                y_pos += 24
        else:
            line_text = self.font_small.render("No system messages yet.", True, self.colors['text'])
            self.screen.blit(line_text, (70, y_pos))
            y_pos += 24

        y_pos = 230

        history_header = self.font_medium.render("Command History:", True, self.colors['highlight'])
        self.screen.blit(history_header, (50, y_pos))
        y_pos += 35

        if self.command_history:
            for i, (cmd, resp) in enumerate(zip(self.command_history, self.responses)):
                if i >= self.max_history:
                    break

                truncated_cmd = self.truncate_text(
                    f"You: {cmd}",
                    self.font_small,
                    self.max_text_width
                )
                cmd_bg = pygame.Rect(60, y_pos, self.width - 120, 25)
                pygame.draw.rect(self.screen, self.colors['command_bg'], cmd_bg)
                cmd_text = self.font_small.render(truncated_cmd, True, self.colors['text'])
                self.screen.blit(cmd_text, (70, y_pos + 3))
                y_pos += 28

                truncated_resp = self.truncate_text(
                    f"Assistant: {resp}",
                    self.font_small,
                    self.max_text_width
                )
                resp_bg = pygame.Rect(60, y_pos, self.width - 120, 25)
                pygame.draw.rect(self.screen, self.colors['response_bg'], resp_bg)
                resp_text = self.font_small.render(truncated_resp, True, self.colors['info'])
                self.screen.blit(resp_text, (70, y_pos + 3))
                y_pos += 32
        else:
            text = self.font_small.render(
                "No commands yet. Type a destination below.",
                True,
                self.colors['text']
            )
            self.screen.blit(text, (70, y_pos))

        pygame.draw.rect(self.screen, self.colors['input_bg'], self.input_rect)

        if len(self.input_text) > 0:
            display_text = self.truncate_text(
                self.input_text,
                self.font_medium,
                self.input_rect.width - 20
            )
            input_surface = self.font_medium.render(display_text, True, self.colors['input_text'])
            self.screen.blit(input_surface, (self.input_rect.x + 10, self.input_rect.y + 5))

        prompt_text = self.font_small.render("Enter destination command:", True, self.colors['highlight'])
        self.screen.blit(prompt_text, (50, 470))

        instructions = [
            "Example commands: 'A-01', '1', '2', 'A03'",
            "Press ENTER to send command",
            "ESC to quit"
        ]

        y_pos = 550
        for instruction in instructions:
            text = self.font_small.render(instruction, True, self.colors['text'])
            self.screen.blit(text, (50, y_pos))
            y_pos += 22

    def process_and_execute_command(self, command: str):
        self.command_history.insert(0, command)

        try:
            selected_target = self.ros_node.selector.select_target(command)

            if selected_target is not None:
                self.ros_node.publish_target(selected_target)
                response_text = f'{selected_target}로 이해했습니다. 미션을 시작합니다.'
            else:
                response_text = '입력을 해석하지 못했습니다. 예: A-01, 2, A-03'

            self.responses.insert(0, response_text)

            if len(self.command_history) > self.max_history:
                self.command_history.pop()
            if len(self.responses) > self.max_history:
                self.responses.pop()

        except Exception as e:
            print(f"Error processing mission command: {e}")
            self.responses.insert(0, f'오류 발생: {str(e)}')

            if len(self.responses) > self.max_history:
                self.responses.pop()

    def run(self):
        print("Starting drone mission chat UI. Type commands and press ENTER.")
        print("Press ESC to exit.")

        clock = pygame.time.Clock()
        running = True

        time.sleep(1.0)

        try:
            while running:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False

                    elif event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_ESCAPE:
                            running = False

                        elif event.key == pygame.K_RETURN and self.input_text:
                            command = self.input_text
                            self.input_text = ""

                            threading.Thread(
                                target=self.process_and_execute_command,
                                args=(command,),
                                daemon=True
                            ).start()

                        elif event.key == pygame.K_BACKSPACE:
                            self.input_text = self.input_text[:-1]

                    elif event.type == pygame.TEXTINPUT:
                        self.input_text += event.text

                self.render_ui()
                pygame.display.flip()
                clock.tick(30)

        finally:
            print("Shutting down UI...")
            pygame.quit()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = MissionUiBridge()

        thread = threading.Thread(target=lambda: rclpy.spin(node))
        thread.daemon = True
        thread.start()

        gui = LLMInterface(node)
        gui.run()

    except Exception as e:
        print(f"Error: {e}")

    finally:
        rclpy.shutdown()
        print("UI terminated.")


if __name__ == '__main__':
    main()