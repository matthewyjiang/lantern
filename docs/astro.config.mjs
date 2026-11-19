// @ts-check
import { defineConfig } from 'astro/config';
import starlight from '@astrojs/starlight';
import starlightLinksValidator from 'starlight-links-validator';

// Served by GitHub Pages as a project site: https://matthewyjiang.github.io/lantern/.
// Links in Markdown don't get `base` added for you, so write them as /lantern/... .
export default defineConfig({
	site: 'https://matthewyjiang.github.io',
	base: '/lantern',
	integrations: [
		starlight({
			title: 'Lantern',
			description:
				'A small RViz stand-in in Rust. Draws point clouds, TF, poses, a robot model, objects and camera images from any Foxglove WebSocket server.',
			logo: { src: './src/assets/lantern-mark.svg' },
			favicon: '/favicon.svg',
			social: [{ icon: 'github', label: 'GitHub', href: 'https://github.com/matthewyjiang/lantern' }],
			customCss: ['./src/styles/theme.css'],
			// Fails the build on a broken internal link or #anchor.
			plugins: [starlightLinksValidator()],
			sidebar: [
				{
					label: 'Start here',
					items: ['getting-started', 'connect'],
				},
				{
					label: 'Guides',
					items: ['guides/publishing', 'guides/robot-model', 'guides/objects', 'guides/layout', 'guides/settings'],
				},
				{
					label: 'Reference',
					items: ['reference/topics', 'reference/replay-server', 'reference/architecture', 'reference/performance'],
				},
			],
		}),
	],
});
