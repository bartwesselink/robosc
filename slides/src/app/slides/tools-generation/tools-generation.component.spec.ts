import { ComponentFixture, TestBed } from '@angular/core/testing';

import { OverviewGenerationComponent } from './tools-generation.component';

describe('OverviewGenerationComponent', () => {
  let component: OverviewGenerationComponent;
  let fixture: ComponentFixture<OverviewGenerationComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ OverviewGenerationComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(OverviewGenerationComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
