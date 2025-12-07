import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  Svg: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Robot Simulation',
    Svg: require('@site/static/img/undraw_robot_simulation_advanced.svg').default,
    description: (
      <>
        Learn to simulate robots in realistic environments using Gazebo and ROS.
        Understand physics engines, collision detection, and sensor modeling.
      </>
    ),
  },
  {
    title: 'AI Integration',
    Svg: require('@site/static/img/undraw_ai_neural_network.svg').default,
    description: (
      <>
        Master the integration of AI algorithms with physical robots. Learn path planning,
        computer vision, and machine learning for robotics applications.
      </>
    ),
  },
  {
    title: 'Humanoid Robotics',
    Svg: require('@site/static/img/undraw_advanced_humanoid.svg').default,
    description: (
      <>
        Explore advanced topics in humanoid robotics including locomotion,
        balance control, and human-robot interaction.
      </>
    ),
  },
];

function Feature({title, Svg, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
