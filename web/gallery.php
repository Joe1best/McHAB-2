<?
$title = "Gallery";
$css = array();
$js = array();
require('./templates/start.php');
?>

<section class="inner">
	<h1>Gallery</h1>

	<div class="gallery clearfix">
		<ul class="image-list">
			<li>
				<a title="title" href="img/test.jpg" rel="Gallery Images">
					<img src="img/test_thumb.jpg" alt="title"/>
				</a>	
			</li>
		</ul>
	</div>

</section>

<?
require('./templates/end.php');
?>