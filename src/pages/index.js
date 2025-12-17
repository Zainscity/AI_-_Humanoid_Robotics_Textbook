import React from 'react';
import clsx from 'clsx';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './index.module.css';
import { motion } from 'framer-motion';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <motion.header
      className={clsx('hero hero--primary', styles.heroBanner)}
      initial={{ opacity: 0 }}
      animate={{ opacity: 1 }}
      transition={{ duration: 1 }}
    >
      <div className="container">
        <motion.h1 
          className="hero__title"
          initial={{ y: -50, opacity: 0 }}
          animate={{ y: 0, opacity: 1 }}
          transition={{ duration: 0.5, delay: 0.5 }}
        >
          {siteConfig.title}
        </motion.h1>
        <motion.p 
          className="hero__subtitle"
          initial={{ y: 50, opacity: 0 }}
          animate={{ y: 0, opacity: 1 }}
          transition={{ duration: 0.5, delay: 1 }}
        >
          {siteConfig.tagline}
        </motion.p>
        <motion.div 
          className={styles.buttons}
          initial={{ opacity: 0 }}
          animate={{ opacity: 1 }}
          transition={{ duration: 0.5, delay: 1.5 }}
        >
          <Link
            className="button button--secondary button--lg"
            to="/docs/">
            Get Started
          </Link>
        </motion.div>
      </div>
    </motion.header>
  );
}

function FeaturesSection() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          <div className={clsx('col col--4')}>
            <div className="text--center">
              <img src="data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAGQAAABkCAYAAABw4pVUAAAAOklEQVR42u3BAQEAAACCoP6v_MolEciAEMhACIQQCEEIRCACEQghEIIgBIIQAhGIQAgCEYhACIIQCAEhEIIgBIIgBJaHD3+2A8Fyo+0aAAAAAElFTkSuQmCC" className={styles.featureImage} alt="RAG Chatbot" />
            </div>
            <h3>Interactive RAG Chatbot</h3>
            <p>Ask questions and get instant answers from the book's content with our Retrieval-Augmented Generation chatbot.</p>
          </div>
          <div className={clsx('col col--4')}>
            <div className="text--center">
              <img src="data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAGQAAABkCAYAAABw4pVUAAAAOklEQVR42u3BAQEAAACCoP6v_MolEciAEMhACIQQCEEIRCACEQghEIIgBIIQAhGIQAgCEYhACIIQCAEhEIIgBIIgBJaHD3+2A8Fyo+0aAAAAAElFTkSuQmCC" className={styles.featureImage} alt="Modules" />
            </div>
            <h3>Comprehensive Modules</h3>
            <p>Explore a wide range of topics, from the foundations of AI to advanced robotics and embodied agents.</p>
          </div>
          <div className={clsx('col col--4')}>
            <div className="text--center">
              <img src="data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAGQAAABkCAYAAABw4pVUAAAAOklEQVR42u3BAQEAAACCoP6v_MolEciAEMhACIQQCEEIRCACEQghEIIgBIIQAhGIQAgCEYhACIIQCAEhEIIgBIIgBJaHD3+2A8Fyo+0aAAAAAElFTkSuQmCC" className={styles.featureImage} alt="Hands-on Projects" />
            </div>
            <h3>Hands-on Projects</h3>
            <p>Apply your knowledge with practical, hands-on projects that bring the concepts to life.</p>
          </div>
        </div>
      </div>
    </section>
  );
}

function CallToActionSection() {
  return (
    <section className={styles.callToAction}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <div className="text--center">
              <h2>Ready to Dive In?</h2>
              <p>Start learning about the exciting world of Physical AI and Humanoid Robotics today!</p>
              <Link
                className="button button--primary button--lg"
                to="/docs/">
                Explore the Book
              </Link>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into a meta tag in <head />">
      <HomepageHeader />
      <main>
        <FeaturesSection />
        <CallToActionSection />
      </main>
    </Layout>
  );
}